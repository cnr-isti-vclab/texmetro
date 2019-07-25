#include "measure.h"
#include "types.h"
#include "gl_utils.h"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <fstream>

static TexImageInfo ComputeTexImageInfo(Mesh& m, const std::vector<Mesh::FacePointer>& faces, int width, int height);


static const char *vs_text_checker[] = {
    "#version 430 core                                           \n"
    "                                                            \n"
    "in vec2 position;                                           \n"
    "in uint chart_id;                                           \n"
    "out flat uint chartId;                                      \n"
    "                                                            \n"
    "void main(void)                                             \n"
    "{                                                           \n"
    "    vec2 p = 2.0 * position - vec2(1.0, 1.0);               \n"
    "    gl_Position = vec4(p, 0.5, 1.0);                        \n"
    "    chartId = chart_id;                                     \n"
    "}                                                           \n"
};

static const char *fs_text_checker[] = {
    "#version 430 core                                                                \n"
    "                                                                                 \n"
    "layout (r32ui) uniform uimage2D imgbuf;                                          \n"
    "layout (r32ui) uniform uimage2D idbuf;                                           \n"
    "in flat uint chartId;                                                            \n"
    "out vec4 color;                                                                  \n"
    "                                                                                 \n"
    "void main(void)                                                                  \n"
    "{                                                                                \n"
    "    color = vec4(1.0, 1.0, 1.0, 1.0);                                            \n"
    "    imageAtomicAdd(imgbuf, ivec2(gl_FragCoord.xy), 1);                           \n"
    "    imageStore(idbuf, ivec2(gl_FragCoord.xy), uvec4(chartId));                   \n"
    "}                                                                                \n"
};


static int FacesByTextureIndex(const Mesh& m, const std::vector<Chart>& atlas, std::vector<std::vector<Mesh::FacePointer>>& fv)
{
    fv.clear();
    fv.resize(m.texsizes.size());

    for (auto& chart : atlas)
        if (chart.sid < (int) m.texsizes.size())
            for (auto fp : chart.fpv)
                fv[chart.sid].push_back(fp);

    return fv.size();
}

using namespace vcg;

MeshInfo ComputeMeshInfo(Mesh& m)
{
    tri::UpdateTopology<Mesh>::FaceFace(m);

    MeshInfo minfo;

    minfo.fn = m.FN();
    minfo.vn = m.VN();

    tri::Clean<Mesh>::CountEdgeNum(m, minfo.en, minfo.en_b, minfo.nme);


    minfo.nmv = tri::Clean<Mesh>::CountNonManifoldVertexFF(m);
    minfo.cc = tri::Clean<Mesh>::CountConnectedComponents(m);
    minfo.bl = -1;
    minfo.g = -1;
    if (minfo.nmv + minfo.nme == 0) {
        minfo.bl = tri::Clean<Mesh>::CountHoles(m);
        minfo.g = tri::Clean<Mesh>::MeshGenus(m);
    }

    return minfo;
}

AtlasInfo ComputeAtlasInfo(Mesh& m, const std::vector<Chart>& atlas, const MeshInfo& minfo)
{
    AtlasInfo ainfo = {};

    // seam edges that are not boundary edges
    ainfo.en_seam = (std::accumulate(atlas.begin(), atlas.end(), 0, [](int a, const Chart& chart) { return a + chart.boundaryCount; }) - minfo.en_b) / 2;

    // number of chart
    ainfo.nc = atlas.size();

    // number of null chart
    ainfo.nnc = std::count_if(atlas.begin(), atlas.end(), [](const Chart& chart) { return chart.areaUV == 0; });

    std::vector<double> f3D(m.FN(), 0); // face area 3D
    std::vector<double> fUV(m.FN(), 0); // face area uv (signed)

    double totalArea3D = 0;
    double mappedArea3D = 0;
    double mappedAreaUV = 0;
    for (auto& f : m.face) {
        int fi = tri::Index(m, f);
        f3D[fi] = DistortionWedge::Area3D(&f);
        fUV[fi] = DistortionWedge::AreaUV(&f);
        totalArea3D += f3D[fi];
        if (f3D[fi] != 0 && fUV[fi] != 0) {
            mappedArea3D += f3D[fi];
            mappedAreaUV += std::abs(fUV[fi]);
        }
    }

    ainfo.mpa = mappedArea3D / totalArea3D;

    ainfo.nfolds = 0;
    for (auto& chart : atlas) {
        for (auto fp : chart.fpv) {
            int fi = tri::Index(m, fp);
            if (std::signbit(fUV[fi]) != std::signbit(chart.areaUV)) {
                ainfo.nfolds++;
            }
        }
    }

    // Histogram?
    {
        double globalScale = mappedAreaUV / mappedArea3D;
        for (auto& f : m.face) {
            // quality is the texel allocation per face area
            int fi = tri::Index(m, f);
            double area3D = f3D[fi];
            double areaUV = fUV[fi];
            if (area3D > 0 && areaUV != 0) {
                double localScale = std::abs(areaUV) / area3D;
                double q = localScale / globalScale;
                f.Q() = q;
            } else {
                f.Q() = 0;
            }
        }

        tri::Stat<Mesh>::ComputePerFaceQualityDistribution(m, ainfo.sfDistrib);

        double sf_perc1 = ainfo.sfDistrib.Percentile(0.01);
        double sf_perc99 = ainfo.sfDistrib.Percentile(0.99);

        ainfo.sfDistrib.Clear();

        for (auto& f : m.face) {
            if (f.Q() >= sf_perc1 && f.Q() <= sf_perc99)
                ainfo.sfDistrib.Add(f.Q());
        }

    }

    ainfo.qcHist.Clear();
    ainfo.qcHist.SetRange(1.0, 10.0, 1000);
    {
        std::vector<double> qcv(m.FN(), std::numeric_limits<double>::max());
        for (auto& f : m.face) {
            int fi = tri::Index(m, f);
            if (f3D[fi] != 0 && fUV[fi] != 0) {
                Eigen::Matrix2d jf = DistortionWedge::mappingTransform2D(f);
                double bcplus  = std::pow(jf(0, 1) + jf(1, 0), 2.0);
                double bcminus = std::pow(jf(0, 1) - jf(1, 0), 2.0);
                double adplus  = std::pow(jf(0, 0) + jf(1, 1), 2.0);
                double adminus = std::pow(jf(0, 0) - jf(1, 1), 2.0);
                double s_min = 0.5 * std::abs(std::sqrt(bcplus + adminus) - std::sqrt(bcminus + adplus));
                double s_max = 0.5 * (std::sqrt(bcplus + adminus) + std::sqrt(bcminus + adplus));

                qcv[fi] = s_max / s_min;
                if (std::isfinite(qcv[fi]))
                    ainfo.qcHist.Add(qcv[fi], f3D[fi]);
            }
        }

        double minPerc = ainfo.qcHist.Percentile(0.01);
        double maxPerc = ainfo.qcHist.Percentile(0.99);

        ainfo.qcHist.Clear();
        ainfo.qcHist.SetRange(1.0, 10.0, 1000);
        for (auto& f : m.face) {
            int fi = tri::Index(m, f);
            if (f3D[fi] != 0 && fUV[fi] != 0) {
                if (std::isfinite(qcv[fi]) && (minPerc < qcv[fi]) && (maxPerc > qcv[fi]))
                    ainfo.qcHist.Add(qcv[fi], f3D[fi]);
            }
        }
    }

    ainfo.mipTextureInfo = ComputeTexImageInfoAtMipLevels(m, atlas);

    // occupancy
    long totalFragments = 0;
    long usedFragments = 0;
    for (auto& v : ainfo.mipTextureInfo) {
        for (unsigned k = 0; k < v.size(); ++k) {
            if (v[k].totalFragments > 0) {
                totalFragments += (v[k].w * v[k].h);
                usedFragments += v[k].totalFragments - v[k].lostFragments;
                break;
            }
        }
    }
    ainfo.occupancy = usedFragments / (double) totalFragments;

    return ainfo;
}

std::vector<std::vector<TexImageInfo>> ComputeTexImageInfoAtMipLevels(Mesh& m, const std::vector<Chart>& atlas)
{
    constexpr int MIN_DIM = 1;

    std::vector<std::vector<Mesh::FacePointer>> facesByTexture;
    int ntex = FacesByTextureIndex(m, atlas, facesByTexture);

    std::vector<std::vector<TexImageInfo>> perTextureMipInfo;
    for (int i = 0; i < ntex; ++i) {
        std::vector<TexImageInfo> mipInfoVec;
        int tw = m.texsizes[i].w;
        int th = m.texsizes[i].h;

        // normalize uvs before rendering
        for (auto fp : facesByTexture[i]) {
            for (int j = 0; j < 3; ++j) {
                fp->WT(j).U() /= m.texsizes[i].w;
                fp->WT(j).V() /= m.texsizes[i].h;
            }
        }

        while (std::min(tw, th) >= MIN_DIM) {
            TexImageInfo mipInfo = ComputeTexImageInfo(m, facesByTexture[i], tw, th);
            mipInfoVec.push_back(mipInfo);
            tw /= 2;
            th /= 2;
        }
        perTextureMipInfo.push_back(mipInfoVec);

        // de-normalize uvs
        for (auto fp : facesByTexture[i]) {
            for (int j = 0; j < 3; ++j) {
                fp->WT(j).U() *= m.texsizes[i].w;
                fp->WT(j).V() *= m.texsizes[i].h;
            }
        }
    }

    return perTextureMipInfo;
}

static const int BitClear = 1;
static const int BitSet = 1;

inline int Check3x3(unsigned *buffer, int row, int col, int width, int height)
{
    int mask = 0;
    if (row == 0 || row == height - 1)
        mask |= BitClear;
    if (col == 0 || col == width - 1)
        mask |= BitSet;
    for (int i = std::max(row-1, 0); i < std::min(row+2, height); ++i) {
        for (int j = std::max(col-1, 0); j < std::min(col+2, width); ++j) {
            if (buffer[i * width + j] == 0)
                mask |= BitClear;
            else
                mask |= BitSet;
        }
    }
    return mask;
}

inline bool Clash3x3(unsigned *buffer, int row, int col, int width, int height)
{
    unsigned id = buffer[row*width + col];
    for (int i = std::max(row-1, 0); i < std::min(row+2, height); ++i) {
        for (int j = std::max(col-1, 0); j < std::min(col+2, width); ++j) {
            unsigned buff_id = buffer[i * width + j];
            if (i != row && j != col && (buff_id != 0 && buff_id != id))
                return true;
        }
    }
    return false;
}

static TexImageInfo ComputeTexImageInfo(Mesh& m, const std::vector<Mesh::FacePointer>& faces, int width, int height)
{
    (void) m;
    assert(sizeof(unsigned) == 4);

    // Create a hidden window
    GLFWwindow *parentWindow = glfwGetCurrentContext();
    bool sharedContext = (parentWindow != nullptr);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);

    GLFWwindow *window = glfwCreateWindow(512, 512, "Window", nullptr, parentWindow);
    if (!window)
    {
        std::cerr << "Failed to create window or context" << std::endl;
        std::exit(-1);
    }
    glfwMakeContextCurrent(window);

    glewExperimental = GL_TRUE;
    GLenum err = glewInit();
    if (err)
    {
        std::cerr << "glew init error " << glewGetErrorString(err) << std::endl;
        std::exit(-1);
    }
    glGetError();

    int fbw, fbh;
    glfwGetFramebufferSize(window, &fbw, &fbh);

    GLuint vao;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    GLint program = CompileShaders(vs_text_checker, fs_text_checker);
    glUseProgram(program);

    GLuint vertexbuf;
    glGenBuffers(1, &vertexbuf);

    glBindBuffer(GL_ARRAY_BUFFER, vertexbuf);
    glBufferData(GL_ARRAY_BUFFER, faces.size()*9*sizeof(float), NULL, GL_STATIC_DRAW);
    float *p = (float *) glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    for (auto fptr : faces) {
        for (int i = 0; i < 3; ++i) {
            *p++ = fptr->cWT(i).U();
            *p++ = fptr->cWT(i).V();

            unsigned int *pp = (unsigned int *) p;
            *pp = (unsigned int) fptr->cid + 1; // the 'background' of the id buffer will be filled with 0s

            p++;
        }
    }
    glUnmapBuffer(GL_ARRAY_BUFFER);

    GLint pos_location = glGetAttribLocation(program, "position");
    glVertexAttribPointer(pos_location, 2, GL_FLOAT, GL_FALSE, 3*sizeof(float), 0);
    glEnableVertexAttribArray(pos_location);

    GLint pos_id = glGetAttribLocation(program, "chart_id");
    glVertexAttribPointer(pos_id, 1, GL_UNSIGNED_INT, GL_FALSE, 3*sizeof(float), (const GLvoid *) (2*sizeof(float)));
    glEnableVertexAttribArray(pos_id);

    p = nullptr;
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    unsigned *sb = new unsigned[width*height](); // zero initialize

    // Create textures to store fragment writes

    // Texture to count overdraws
    constexpr int imgbuf_unit = 0;
    GLint loc_imgbuf = glGetUniformLocation(program, "imgbuf");
    glUniform1i(loc_imgbuf, imgbuf_unit);

    GLuint tex;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);

    glTexStorage2D(GL_TEXTURE_2D, 1, GL_R32UI, width, height);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RED_INTEGER, GL_UNSIGNED_INT, sb);

    glBindImageTexture(imgbuf_unit, tex, 0, GL_FALSE, 0, GL_READ_WRITE, GL_R32UI);

    // Texture to store marks. The texture is initially zero-initialized, and each fragment
    // write checks if the stored pixel value is either zero, an 'invalid' value (0xffffffff), or a non-zero chart id.
    // If it reads zero, it writes its value to the pixel (with atomic compare and swap). Otherwise, if the comparison failed
    // and the returned value is invalid, nothing happens. If the returned value is another chart id instead, then it is checked
    // against the chart id of the fragment, and if it is different it means that two different regions overlap on the pizxel, and
    // is therefore set to the invalid value 0xffffffff
    // Note that since 0 is a perfectly valid id, the id of each face as a vertex attribute is incremented by one
    constexpr int idbuf_unit = 1;
    GLint loc_idbuf = glGetUniformLocation(program, "idbuf");
    glUniform1i(loc_idbuf, idbuf_unit);
    GLuint tex_id;
    glGenTextures(1, &tex_id);
    glBindTexture(GL_TEXTURE_2D, tex_id);
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_R32UI, width, height);

    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RED_INTEGER, GL_UNSIGNED_INT, sb);
    glBindImageTexture(idbuf_unit, tex_id, 0, GL_FALSE, 0, GL_READ_WRITE, GL_R32UI);

    GLuint fbo;
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    glFramebufferParameteri(GL_FRAMEBUFFER, GL_FRAMEBUFFER_DEFAULT_WIDTH, width);
    glFramebufferParameteri(GL_FRAMEBUFFER, GL_FRAMEBUFFER_DEFAULT_HEIGHT, height);

    glViewport(0, 0, width, height);
    glScissor(0, 0, width, height);

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_STENCIL_TEST);

    glClearColor(0.0f, 1.0f, 0.0f, 1.0f);

    glDrawBuffer(GL_COLOR_ATTACHMENT0);

    glClear(GL_COLOR_BUFFER_BIT);
    glfwPollEvents();

    glDrawArrays(GL_TRIANGLES, 0, faces.size()*3);

    glMemoryBarrier(GL_ALL_BARRIER_BITS);

    glBindTexture(GL_TEXTURE_2D, tex);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, sb);

    unsigned *mb = new unsigned[width*height];

    glBindTexture(GL_TEXTURE_2D, tex_id);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, mb);

    CheckGLError();

    //glReadBuffer(GL_BACK);
    //glReadPixels(0, 0, width, height, GL_STENCIL_INDEX, GL_UNSIGNED_BYTE, sb);

    TexImageInfo mipInfo = {};
    mipInfo.w = width;
    mipInfo.h = height;
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            int k = i * width + j;
            int n = sb[k]; // stencil value
            int kernelMask = Check3x3(sb, i, j, width, height);
            if (n > 0) {
                mipInfo.totalFragments += n;
                mipInfo.totalFragments_bilinear++;
                if ((n > 1)) {
                    mipInfo.overwrittenFragments++;
                    mipInfo.lostFragments += (n - 1);
                }
                if (Clash3x3(mb, i, j, width, height))
                    mipInfo.fragmentClashes++;

                if (kernelMask &= BitClear)
                    mipInfo.boundaryFragments++;
            } else {
                if (kernelMask &= BitSet)
                    mipInfo.totalFragments_bilinear++;
            }
        }
    }

    glfwPollEvents();

    delete [] sb;
    delete [] mb;

    // clean up
    glUseProgram(0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindVertexArray(0);

    glDeleteBuffers(1, &vertexbuf);
    glDeleteTextures(1, &tex);
    glDeleteTextures(1, &tex_id);
    glDeleteProgram(program);
    glDeleteVertexArrays(1, &vao);

    glfwDestroyWindow(window);

    if (sharedContext) {
        glfwMakeContextCurrent(parentWindow);
    }

    return mipInfo;
}

static std::string JSONField(const char *fname, int fval, int ident = 0)
{
    std::stringstream ss;
    while (ident-- > 0)
        ss << "  ";
    ss << "\"" << fname << "\": " << fval;
    return ss.str();
}

static std::string JSONField(const char *fname, double fval, int ident = 0)
{
    std::stringstream ss;
    while (ident-- > 0)
        ss << "  ";
    ss << "\"" << fname << "\": " << fval;
    return ss.str();
}

static std::string ToJSON(const TexImageInfo& tii, int ident = 0)
{
    std::stringstream ss;

    int ii = ident;
    while (ii-- > 0)
        ss << "  ";
    ss << "{" << std::endl;

    ss << JSONField("rw"                     , tii.w,                       ident + 1) << "," << std::endl;
    ss << JSONField("rh"                     , tii.h,                       ident + 1) << "," << std::endl;
    ss << JSONField("totalFragments"         , tii.totalFragments,          ident + 1) << "," << std::endl;
    ss << JSONField("totalFragments_bilinear", tii.totalFragments_bilinear, ident + 1) << "," << std::endl;
    ss << JSONField("overwrittenFragments"   , tii.overwrittenFragments,    ident + 1) << "," << std::endl;
    ss << JSONField("lostFragments"          , tii.lostFragments,           ident + 1) << "," << std::endl;
    ss << JSONField("fragmentClashes"        , tii.fragmentClashes,         ident + 1) << "," << std::endl;
    ss << JSONField("boundaryFragments"      , tii.boundaryFragments,       ident + 1) << std::endl;

    ii = ident;
    while (ii-- > 0)
        ss << "  ";
    ss << "}";

    return ss.str();
}

static std::string JSONField(const char *fname, const std::vector<TexImageInfo>& vt, int ident = 0)
{
    std::stringstream ss;

    int ii = ident;
    while (ii-- > 0)
        ss << "  ";
    ss << "\"" << fname << "\": [" << std::endl;

    for (std::size_t i = 0; i < vt.size(); ++i) {
        if (i > 0)
            ss << "," << std::endl;
        ss << ToJSON(vt[i], ident + 1);
    }

    ss << std::endl;
    ii = ident;
    while (ii-- > 0)
        ss << "  ";
    ss << "]";

    return ss.str();
}

static std::string JSONField(const char *fname, const std::vector<std::string>& vs, int ident = 0)
{
    std::stringstream ss;

    int ii = ident;
    while (ii-- > 0)
        ss << "  ";
    std::string ws = ss.str();
    ss << "\"" << fname << "\": [" << std::endl;

    for (std::size_t i = 0; i < vs.size(); ++i) {
        if (i > 0)
            ss << "," << std::endl;
        ss << ws << "  \"" << vs[i] << "\"";
    }

    ss << std::endl;
    ii = ident;
    while (ii-- > 0)
        ss << "  ";
    ss << "]";

    return ss.str();
}

static std::string JSONField(const char *fname, const char *str, int ident = 0)
{
    std::stringstream ss;
    while (ident-- > 0)
        ss << "  ";
    ss << "\"" << fname << "\": \"" << str << "\"";
    return ss.str();
}

void WriteJSON(const std::string& filename, const Mesh& m, const MeshInfo& minfo, AtlasInfo &ainfo)
{

    // write all the stats to a json object
    std::ofstream json(filename + ".json");

    json << "{" << std::endl;
    json << JSONField("mesh"                , m.name.c_str(),    1)    << "," << std::endl;
    json << JSONField("fn"                  , minfo.fn,           1)    << "," << std::endl;
    json << JSONField("vn"                  , minfo.vn,           1)    << "," << std::endl;
    json << JSONField("en"                  , minfo.en,           1)    << "," << std::endl;
    json << JSONField("en_b"                , minfo.en_b,         1)    << "," << std::endl;
    json << JSONField("en_seam"             , ainfo.en_seam,      1)    << "," << std::endl;
    json << JSONField("nonmanif_edge"       , minfo.nme,          1)    << "," << std::endl;
    json << JSONField("nonmanif_vert"       , minfo.nmv,          1)    << "," << std::endl;
    json << JSONField("connected_components", minfo.cc ,          1)    << "," << std::endl;
    json << JSONField("boundary_loops"      , minfo.bl ,          1)    << "," << std::endl;
    json << JSONField("genus"               , minfo.g  ,          1)    << "," << std::endl;
    json << JSONField("num_charts"          , ainfo.nc ,          1)    << "," << std::endl;
    json << JSONField("num_null_charts"     , ainfo.nnc,          1)    << "," << std::endl;
    json << JSONField("mapped_fraction"     , ainfo.mpa,          1)    << "," << std::endl;
    json << JSONField("nfolds"              , ainfo.nfolds,       1)    << "," << std::endl;
    json << JSONField("occupancy"           , ainfo.occupancy,    1)    << "," << std::endl;
    json << JSONField("ntex"                , (int) ainfo.mipTextureInfo.size(), 1) << "," << std::endl;
    json << JSONField("textures"            , m.textures,                        1) << "," << std::endl;

    for (std::size_t ntex = 0; ntex < ainfo.mipTextureInfo.size(); ntex++) {
        std::stringstream ss;
        ss << "texture_" << ntex;
        json << JSONField(ss.str().c_str(), ainfo.mipTextureInfo[ntex], 1) << "," << std::endl;
    }

    json << JSONField("qc_dist_perc1" , ainfo.qcHist.Percentile(0.01), 1)     << "," << std::endl;
    json << JSONField("qc_dist_perc10", ainfo.qcHist.Percentile(0.10), 1)     << "," << std::endl;
    json << JSONField("qc_dist_perc20", ainfo.qcHist.Percentile(0.20), 1)     << "," << std::endl;
    json << JSONField("qc_dist_perc30", ainfo.qcHist.Percentile(0.30), 1)     << "," << std::endl;
    json << JSONField("qc_dist_perc40", ainfo.qcHist.Percentile(0.40), 1)     << "," << std::endl;
    json << JSONField("qc_dist_perc50", ainfo.qcHist.Percentile(0.50), 1)     << "," << std::endl;
    json << JSONField("qc_dist_perc60", ainfo.qcHist.Percentile(0.60), 1)     << "," << std::endl;
    json << JSONField("qc_dist_perc70", ainfo.qcHist.Percentile(0.70), 1)     << "," << std::endl;
    json << JSONField("qc_dist_perc80", ainfo.qcHist.Percentile(0.80), 1)     << "," << std::endl;
    json << JSONField("qc_dist_perc90", ainfo.qcHist.Percentile(0.90), 1)     << "," << std::endl;
    json << JSONField("qc_dist_perc99", ainfo.qcHist.Percentile(0.99), 1)     << "," << std::endl;
    json << JSONField("qc_dist_avg"   , ainfo.qcHist.Avg(), 1)                << "," << std::endl;
    json << JSONField("qc_dist_rms"   , ainfo.qcHist.RMS(), 1)                << "," << std::endl;
    json << JSONField("qc_dist_var"   , ainfo.qcHist.Variance(), 1)           << "," << std::endl;
    json << JSONField("qc_dist_stddev", ainfo.qcHist.StandardDeviation(), 1)  << "," << std::endl;

    json << JSONField("sf_perc1" , ainfo.sfDistrib.Percentile(0.01), 1)     << "," << std::endl;
    json << JSONField("sf_perc10", ainfo.sfDistrib.Percentile(0.10), 1)     << "," << std::endl;
    json << JSONField("sf_perc20", ainfo.sfDistrib.Percentile(0.20), 1)     << "," << std::endl;
    json << JSONField("sf_perc30", ainfo.sfDistrib.Percentile(0.30), 1)     << "," << std::endl;
    json << JSONField("sf_perc40", ainfo.sfDistrib.Percentile(0.40), 1)     << "," << std::endl;
    json << JSONField("sf_perc50", ainfo.sfDistrib.Percentile(0.50), 1)     << "," << std::endl;
    json << JSONField("sf_perc60", ainfo.sfDistrib.Percentile(0.60), 1)     << "," << std::endl;
    json << JSONField("sf_perc70", ainfo.sfDistrib.Percentile(0.70), 1)     << "," << std::endl;
    json << JSONField("sf_perc80", ainfo.sfDistrib.Percentile(0.80), 1)     << "," << std::endl;
    json << JSONField("sf_perc90", ainfo.sfDistrib.Percentile(0.90), 1)     << "," << std::endl;
    json << JSONField("sf_perc99", ainfo.sfDistrib.Percentile(0.99), 1)     << "," << std::endl;
    json << JSONField("sf_avg"   , ainfo.sfDistrib.Avg(), 1)                << "," << std::endl;
    json << JSONField("sf_rms"   , ainfo.sfDistrib.RMS(), 1)                << "," << std::endl;
    json << JSONField("sf_var"   , ainfo.sfDistrib.Variance(), 1)           << "," << std::endl;
    json << JSONField("sf_stddev", ainfo.sfDistrib.StandardDeviation(), 1)  << "," << std::endl;

    json << "}" << std::endl;

    json.close();

}
