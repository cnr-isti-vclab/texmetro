#include "measure.h"
#include "mesh.h"
#include "gl_utils.h"
#include "color_consistency.h"

#include <GL/glew.h>

#include <QSurfaceFormat>
#include <QOpenGLContext>
#include <QOffscreenSurface>

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
    "#version 430 core                                                     \n"
    "                                                                      \n"
    "layout (r32ui) uniform uimage2D imgbuf;                               \n"
    "layout (r32ui) uniform uimage2D idbuf;                                \n"
    "in flat uint chartId;                                                 \n"
    "out vec4 color;                                                       \n"
    "                                                                      \n"
    "void main(void)                                                       \n"
    "{                                                                     \n"
    "    color = vec4(1.0, 1.0, 1.0, 1.0);                                 \n"
    "    imageAtomicAdd(imgbuf, ivec2(gl_FragCoord.xy), 1);                \n"
    "    imageStore(idbuf, ivec2(gl_FragCoord.xy), uvec4(chartId));        \n"
    "}                                                                     \n"
};


static int FacesByTextureIndex(const Mesh& m, const std::vector<Chart>& atlas, std::vector<std::vector<Mesh::FacePointer>>& fv)
{
    fv.clear();
    fv.resize(m.texsizes.size());

    for (auto& chart : atlas)
        if (chart.ti < (int) m.texsizes.size())
            for (auto fp : chart.fpv)
                fv[chart.ti].push_back(fp);

    return fv.size();
}

static int CountDegenerateFace(const Mesh& m)
{
    int count_fd = 0;
    for (const auto& f : m.face) {
        if (!f.IsD()) {
            if(f.cV(0) == f.cV(1) || f.cV(0) == f.cV(2) || f.cV(1) == f.cV(2)) {
                count_fd++;
            }
        }
    }
    return count_fd;
}

using namespace vcg;

MeshInfo ComputeMeshInfo(Mesh& m)
{
    tri::UpdateTopology<Mesh>::FaceFace(m);
    tri::UpdateBounding<Mesh>::Box(m);

    double maxBoxDim = m.bbox.Dim()[m.bbox.MaxDim()];
    double normalizationFactor = 1.0 / maxBoxDim;

    MeshInfo minfo = {};

    minfo.fn = m.FN();
    minfo.vn = m.VN();

    tri::Clean<Mesh>::CountEdgeNum(m, minfo.en, minfo.en_b, minfo.nme);


    minfo.vnunref = tri::Clean<Mesh>::CountUnreferencedVertex(m);
    minfo.nmv = tri::Clean<Mesh>::CountNonManifoldVertexFF(m);

    // count duplicate faces
    typedef tri::Clean<Mesh>::SortedTriple Triplet;
    std::vector<Triplet> tvec;
    for (auto& f : m.face) {
        tvec.push_back(Triplet(tri::Index(m, f.V(0)), tri::Index(m, f.V(1)), tri::Index(m, f.V(2)), &f));
    }
    std::sort(tvec.begin(), tvec.end());
    auto it = std::unique(tvec.begin(), tvec.end());
    minfo.fndup = minfo.fn - (it - tvec.begin());

    // count zero-area faces
    minfo.fnzero = 0;
    for (auto& f : m.face) {
        if (DistortionWedge::Area3D(&f) <= 0)
            minfo.fnzero++;
    }

    minfo.fndegen = CountDegenerateFace(m);

    minfo.cc = tri::Clean<Mesh>::CountConnectedComponents(m);
    minfo.g = -1;
    minfo.bl = -1;
    if (minfo.nmv + minfo.nme == 0) {
        minfo.bl = tri::Clean<Mesh>::CountHoles(m);
        minfo.g = tri::Clean<Mesh>::MeshGenus(m.vn - minfo.vnunref, minfo.en, m.fn, minfo.bl, minfo.cc);
    }
    minfo.area = 0;
    minfo.bnd_len = 0;
    for (auto& f : m.face) {
        minfo.area += DistortionWedge::Area3D(&f);
        for (int i = 0; i < 3; ++i) {
            if (face::IsBorder(f, i))
                minfo.bnd_len += DistortionWedge::EdgeLenght3D(&f, i);
        }
    }

    minfo.area_normalized = minfo.area * std::pow(normalizationFactor, 2.0);
    minfo.bnd_len_normalized = minfo.bnd_len * normalizationFactor;

    tri::Clean<Mesh>::OrientCoherentlyMesh(m, minfo.oriented, minfo.orientable);
    tri::UpdateTopology<Mesh>::FaceFace(m);

    return minfo;
}

AtlasInfo ComputeAtlasInfo(Mesh& m, const std::vector<Chart>& atlas)
{
    tri::UpdateTopology<Mesh>::FaceFace(m);

    AtlasInfo ainfo = {};

    // count seams in uv space
    int nme;
    tri::Clean<Mesh>::CountEdgeNum(m, ainfo.en_uv, ainfo.en_uv_b, nme);

    // number of chart
    ainfo.nc = atlas.size();

    // number of null chart
    ainfo.nnc = std::count_if(atlas.begin(), atlas.end(), [](const Chart& chart) { return chart.areaUV == 0; });

    std::vector<double> f3D(m.FN(), 0); // face areas 3D
    std::vector<double> fUV(m.FN(), 0); // face areas uv (signed)

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

    ainfo.area = 0;
    ainfo.bnd_len = 0;
    for (auto& f : m.face) {
        ainfo.area += std::abs(fUV[tri::Index(m, f)]);
        for (int i = 0; i < 3; ++i) {
            if (face::IsBorder(f, i))
                ainfo.bnd_len += DistortionWedge::EdgeLenghtUV(&f, i);
        }
    }

    ainfo.nfolds = 0;
    for (auto& chart : atlas) {
        for (auto fp : chart.fpv) {
            int fi = tri::Index(m, fp);
            if (std::signbit(fUV[fi]) != std::signbit(chart.signedAreaUV)) {
                ainfo.nfolds++;
            }
        }
    }

    ainfo.mipTextureInfo = ComputeTexImageInfoAtMipLevels(m, atlas);

    // occupancy
    long availableFragments = 0;
    long usedFragments = 0;
    for (auto& mti : ainfo.mipTextureInfo) {
        for (unsigned k = 0; k < mti.size(); ++k) {
            if (mti[k].totalFragments > 0) {
                availableFragments += (mti[k].w * mti[k].h);
                usedFragments += mti[k].totalFragments - mti[k].lostFragments;
                break;
            }
        }
    }
    ainfo.occupancy = usedFragments / (double) availableFragments;

    {
        for (auto& f : m.face) {
            int fi = tri::Index(m, f);
            f.Q() = 0;
            if (f3D[fi] != 0 && fUV[fi] != 0) {
                vcg::Point2d x10, x20;
                LocalIsometry(f.P(1) - f.P(0), f.P(2) - f.P(0), x10, x20);
                Eigen::Matrix2d jf = ComputeTransformationMatrix(x10, x20, f.WT(1).P() - f.WT(0).P(), f.WT(2).P() - f.WT(0).P());


                //Eigen::Matrix2d jf = DistortionWedge::mappingTransform2D(f);
                double bcplus  = std::pow(jf(0, 1) + jf(1, 0), 2.0);
                double bcminus = std::pow(jf(0, 1) - jf(1, 0), 2.0);
                double adplus  = std::pow(jf(0, 0) + jf(1, 1), 2.0);
                double adminus = std::pow(jf(0, 0) - jf(1, 1), 2.0);
                double s_min = 0.5 * std::abs(std::sqrt(bcplus + adminus) - std::sqrt(bcminus + adplus));
                double s_max = 0.5 * (std::sqrt(bcplus + adminus) + std::sqrt(bcminus + adplus));
                if (s_max < s_min)
                    std::swap(s_max, s_min);
                double qcd = s_max / s_min;
                assert(qcd >= 1);
                if (std::isfinite(qcd))
                    f.Q() = qcd;
            }
        }

        ainfo.qcd_areaAboveThreshold = 0;
        tri::UpdateSelection<Mesh>::FaceClear(m);
        for (auto& f : m.face) {
            assert(std::isfinite(f.Q()));
            if (f.Q() > 0) {
                if (f.Q() < QUASI_CONFORMAL_DISTORTION_MAX_THRESHOLD)
                    f.SetS();
                else
                    ainfo.qcd_areaAboveThreshold += f3D[tri::Index(m, f)];
            }
        }

        ainfo.qcHistogram.Clear();
        ainfo.qcHistogram.SetRange(0, QUASI_CONFORMAL_DISTORTION_MAX_THRESHOLD, 500000);
        for (auto& f : m.face) {
            if (f.IsS()) {
                ainfo.qcHistogram.Add(f.Q(), DistortionWedge::Area3D(&f));
            }
        }
    }

    {
        long totalTexelCount = 0;
        for (auto& mti : ainfo.mipTextureInfo)
            totalTexelCount += mti[0].w * mti[0].h;

        double realTextureArea = ainfo.occupancy * totalTexelCount;

        double sf3D = 0;
        double sfUV = 0;

        ainfo.sf_areaAboveThreshold = 0;
        for (auto& f : m.face) {
            int fi = tri::Index(m, f);
            double area3D = f3D[fi];
            double areaUV = std::abs(fUV[fi]);
            f.Q() = 0;
            if (area3D > 0 && areaUV > 0) {
                double perc3D = area3D / mappedArea3D;
                double thresholdUV = (perc3D * realTextureArea) * UV_SCALING_RATIO_MAX_THRESHOLD;
                if (areaUV <= thresholdUV) {
                    f.Q() = areaUV / area3D;
                    sf3D += area3D;
                    sfUV += areaUV;
                } else {
                    ainfo.sf_areaAboveThreshold += f3D[tri::Index(m, f)];
                }
            }
        }

        ainfo.sfHistogram.Clear();
        ainfo.sfHistogram.SetRange(0, UV_SCALING_RATIO_MAX_THRESHOLD, 20000000);
        double denom = sfUV / sf3D;
        for (auto& f : m.face)
            if (f.Q() > 0)
                ainfo.sfHistogram.Add(f.Q() / denom, DistortionWedge::Area3D(&f));
    }

    return ainfo;
}

std::vector<std::vector<TexImageInfo>> ComputeTexImageInfoAtMipLevels(Mesh& m, const std::vector<Chart>& atlas)
{
    constexpr int MIN_DIM = 1;

    std::vector<std::vector<Mesh::FacePointer>> facesByTexture;
    int ntex = FacesByTextureIndex(m, atlas, facesByTexture);

    std::vector<std::vector<TexImageInfo>> perTextureMipInfo;
    for (int i = 0; i < ntex; ++i) {
        std::vector<TexImageInfo> texInfoVec;
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
            TexImageInfo texInfo = ComputeTexImageInfo(m, facesByTexture[i], tw, th);
            texInfoVec.push_back(texInfo);
            tw /= 2;
            th /= 2;
        }
        perTextureMipInfo.push_back(texInfoVec);

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
static const int BitSet = 2;

inline int Check3x3(unsigned *buffer, int row, int col, int width, int height)
{
    int mask = 0;
    if (row == 0 || row == height - 1)
        mask |= BitClear;
    if (col == 0 || col == width - 1)
        mask |= BitClear;
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

    QSurfaceFormat format;
    format.setVersion(4, 2);
    format.setProfile(QSurfaceFormat::OpenGLContextProfile::CoreProfile);
    format.setSwapBehavior(QSurfaceFormat::SwapBehavior::SingleBuffer);

    QOpenGLContext context;
    context.setFormat(format);

    if (!context.create()) {
        std::cerr << "Failed to create opengl context" << std::endl;
    }

    {
        QSurfaceFormat f = context.format();
        if (f.majorVersion() != format.majorVersion() || f.minorVersion() != f.minorVersion()) {
            std::cerr << "Warning: Could not create OpenGL context with version "
                      << format.majorVersion() << "." << format.minorVersion() << std::endl;
        }
        if (format.profile() != QSurfaceFormat::OpenGLContextProfile::CoreProfile){
            std::cerr << "Warning: Core OpenGL profile not available" << std::endl;
        }
    }

    QOffscreenSurface surface;
    surface.setFormat(context.format());
    surface.create();

    if (!context.makeCurrent(&surface)) {
        std::cerr << "Failed to create OpenGL context" << std::endl;
        std::exit(-1);
    }

    glewExperimental = GL_TRUE;
    GLenum err = glewInit();
    if (err)
    {
        std::cerr << "Failed to initialize GLEW " << glewGetErrorString(err) << std::endl;
        std::exit(-1);
    }
    glGetError();

    GLint maxTexSize;
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maxTexSize);
    if (std::max(width, height) > maxTexSize) {
        std::cerr << "Warning: texture size exceeds implementation limits" << std::endl;
        TexImageInfo texInfo = {};
        texInfo.w = width;
        texInfo.h = height;
        return texInfo;
    }

    GLuint vao;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    GLint program = CompileShaders(vs_text_checker, fs_text_checker);
    glUseProgram(program);

    CheckGLError();
    GLuint vertexbuf;
    glGenBuffers(1, &vertexbuf);

    CheckGLError();

    glBindBuffer(GL_ARRAY_BUFFER, vertexbuf);
    CheckGLError();
    glBufferData(GL_ARRAY_BUFFER, faces.size()*9*sizeof(float), NULL, GL_STATIC_DRAW);
    CheckGLError();
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
    CheckGLError();
    glUnmapBuffer(GL_ARRAY_BUFFER);

    CheckGLError();
    GLint pos_location = glGetAttribLocation(program, "position");
    glVertexAttribPointer(pos_location, 2, GL_FLOAT, GL_FALSE, 3*sizeof(float), 0);
    glEnableVertexAttribArray(pos_location);

    CheckGLError();
    GLint pos_id = glGetAttribLocation(program, "chart_id");
    glVertexAttribPointer(pos_id, 1, GL_UNSIGNED_INT, GL_FALSE, 3*sizeof(float), (const GLvoid *) (2*sizeof(float)));
    glEnableVertexAttribArray(pos_id);

    CheckGLError();

    p = nullptr;
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    unsigned *sb = new unsigned[width*height](); // zero initialize

    // Create textures to store fragment writes

    // Texture to count overdraws
    constexpr int imgbuf_unit = 0;
    GLint loc_imgbuf = glGetUniformLocation(program, "imgbuf");
    glUniform1i(loc_imgbuf, imgbuf_unit);

    CheckGLError();
    GLuint tex;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);

    glTexStorage2D(GL_TEXTURE_2D, 1, GL_R32UI, width, height);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RED_INTEGER, GL_UNSIGNED_INT, sb);

    glBindImageTexture(imgbuf_unit, tex, 0, GL_FALSE, 0, GL_READ_WRITE, GL_R32UI);

    constexpr int idbuf_unit = 1;
    GLint loc_idbuf = glGetUniformLocation(program, "idbuf");
    glUniform1i(loc_idbuf, idbuf_unit);
    GLuint tex_id;
    glGenTextures(1, &tex_id);
    glBindTexture(GL_TEXTURE_2D, tex_id);
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_R32UI, width, height);

    CheckGLError();
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RED_INTEGER, GL_UNSIGNED_INT, sb);
    glBindImageTexture(idbuf_unit, tex_id, 0, GL_FALSE, 0, GL_READ_WRITE, GL_R32UI);

    GLuint fbo;
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    glFramebufferParameteri(GL_FRAMEBUFFER, GL_FRAMEBUFFER_DEFAULT_WIDTH, width);
    glFramebufferParameteri(GL_FRAMEBUFFER, GL_FRAMEBUFFER_DEFAULT_HEIGHT, height);

    glViewport(0, 0, width, height);
    glScissor(0, 0, width, height);

    CheckGLError();
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_STENCIL_TEST);

    glClearColor(0.0f, 1.0f, 0.0f, 1.0f);

    glDrawBuffer(GL_COLOR_ATTACHMENT0);

    glClear(GL_COLOR_BUFFER_BIT);

    glDrawArrays(GL_TRIANGLES, 0, faces.size()*3);

    glMemoryBarrier(GL_ALL_BARRIER_BITS);

    glBindTexture(GL_TEXTURE_2D, tex);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, sb);

    CheckGLError();
    unsigned *mb = new unsigned[width*height];

    glBindTexture(GL_TEXTURE_2D, tex_id);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RED_INTEGER, GL_UNSIGNED_INT, mb);

    CheckGLError();

    TexImageInfo texInfo = {};
    texInfo.w = width;
    texInfo.h = height;
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            int k = i * width + j;
            int n = sb[k]; // stencil value
            int checkMask = Check3x3(sb, i, j, width, height);
            if (n > 0) {
                texInfo.totalFragments += n;
                texInfo.totalFragments_bilinear++;
                if ((n > 1)) {
                    texInfo.overwrittenFragments++;
                    texInfo.lostFragments += (n - 1);
                }
                if (Clash3x3(mb, i, j, width, height))
                    texInfo.fragmentClashes++;

                if (checkMask &= BitClear)
                    texInfo.boundaryFragments++;
            } else {
                if (checkMask &= BitSet)
                    texInfo.totalFragments_bilinear++;
            }
        }
    }

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

    return texInfo;
}

static std::string JSONField(const char *fname, int fval, int indentation = 0)
{
    std::stringstream ss;
    while (indentation-- > 0)
        ss << "  ";
    ss << "\"" << fname << "\": " << fval;
    return ss.str();
}

static std::string JSONField(const char *fname, bool fval, int indentation = 0)
{
    std::stringstream ss;
    while (indentation-- > 0)
        ss << "  ";
    ss << "\"" << fname << "\": " << (fval ? "true" : "false");
    return ss.str();
}

static std::string JSONField(const char *fname, double fval, int indentation = 0)
{
    std::stringstream ss;
    while (indentation-- > 0)
        ss << "  ";
    ss << "\"" << fname << "\": " << fval;
    return ss.str();
}

static std::string ToJSON(const TexImageInfo& tii, int indentation = 0)
{
    std::stringstream ss;

    int ii = indentation;
    while (ii-- > 0)
        ss << "  ";
    ss << "{" << std::endl;

    ss << JSONField("rw"                     , tii.w,                       indentation + 1) << "," << std::endl;
    ss << JSONField("rh"                     , tii.h,                       indentation + 1) << "," << std::endl;
    ss << JSONField("totalFragments"         , tii.totalFragments,          indentation + 1) << "," << std::endl;
    ss << JSONField("totalFragments_bilinear", tii.totalFragments_bilinear, indentation + 1) << "," << std::endl;
    ss << JSONField("overwrittenFragments"   , tii.overwrittenFragments,    indentation + 1) << "," << std::endl;
    ss << JSONField("lostFragments"          , tii.lostFragments,           indentation + 1) << "," << std::endl;
    ss << JSONField("fragmentClashes"        , tii.fragmentClashes,         indentation + 1) << "," << std::endl;
    ss << JSONField("boundaryFragments"      , tii.boundaryFragments,       indentation + 1) << std::endl;

    ii = indentation;
    while (ii-- > 0)
        ss << "  ";
    ss << "}";

    return ss.str();
}

static std::string JSONField(const char *fname, const std::vector<TexImageInfo>& vt, int indentation = 0)
{
    std::stringstream ss;

    int ii = indentation;
    while (ii-- > 0)
        ss << "  ";
    ss << "\"" << fname << "\": [" << std::endl;

    for (std::size_t i = 0; i < vt.size(); ++i) {
        if (i > 0)
            ss << "," << std::endl;
        ss << ToJSON(vt[i], indentation + 1);
    }

    ss << std::endl;
    ii = indentation;
    while (ii-- > 0)
        ss << "  ";
    ss << "]";

    return ss.str();
}

static std::string JSONField(const char *fname, const std::vector<std::string>& vs, int indentation = 0)
{
    std::stringstream ss;

    int ii = indentation;
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
    ii = indentation;
    while (ii-- > 0)
        ss << "  ";
    ss << "]";

    return ss.str();
}

static std::string JSONField(const char *fname, const char *str, int indentation = 0)
{
    std::stringstream ss;
    while (indentation-- > 0)
        ss << "  ";
    ss << "\"" << fname << "\": \"" << str << "\"";
    return ss.str();
}

void WriteJSON(const std::string& filename, const Mesh& m, const MeshInfo& minfo, AtlasInfo &ainfo, ColorConsistencyInfo& cci)
{
    // write all the stats to a json object
    std::ofstream json(filename);
    if (!json.is_open()) {
        std::cerr << "Error opening file " << filename << ": " << std::strerror(errno) << std::endl;
        std::exit(-1);
    }

    json << "{" << std::endl;

    json << JSONField("mesh"                   , m.name.c_str(),           1)    << "," << std::endl;
    json << JSONField("fn"                     , minfo.fn,                 1)    << "," << std::endl;
    json << JSONField("vn"                     , minfo.vn,                 1)    << "," << std::endl;
    json << JSONField("en"                     , minfo.en,                 1)    << "," << std::endl;
    json << JSONField("en_b"                   , minfo.en_b,               1)    << "," << std::endl;
    json << JSONField("area"                   , minfo.area,               1)    << "," << std::endl;
    json << JSONField("boundary_len"           , minfo.bnd_len,            1)    << "," << std::endl;
    json << JSONField("area_normalized"        , minfo.area_normalized,    1)    << "," << std::endl;
    json << JSONField("boundary_len_normalized", minfo.bnd_len_normalized, 1)    << "," << std::endl;

    json << JSONField("boundary_loops"      , minfo.bl ,          1)    << "," << std::endl;
    json << JSONField("nonmanif_edge"       , minfo.nme,          1)    << "," << std::endl;
    json << JSONField("nonmanif_vert"       , minfo.nmv,          1)    << "," << std::endl;
    json << JSONField("connected_components", minfo.cc ,          1)    << "," << std::endl;
    json << JSONField("genus"               , minfo.g  ,          1)    << "," << std::endl;
    json << JSONField("fndup"               , minfo.fndup,        1)    << "," << std::endl;
    json << JSONField("fnzero"              , minfo.fnzero,       1)    << "," << std::endl;
    json << JSONField("fndegen"             , minfo.fndegen,      1)    << "," << std::endl;
    json << JSONField("vnunref"             , minfo.vnunref,      1)    << "," << std::endl;
    json << JSONField("oriented"            , minfo.oriented,     1)    << "," << std::endl;
    json << JSONField("orientable"          , minfo.orientable,   1)    << "," << std::endl;

    json << JSONField("en_uv"               , ainfo.en_uv,        1)    << "," << std::endl;
    json << JSONField("en_uv_b"             , ainfo.en_uv_b,      1)    << "," << std::endl;
    json << JSONField("area_uv"             , ainfo.area,         1)    << "," << std::endl;
    json << JSONField("boundary_len_uv"     , ainfo.bnd_len,      1)    << "," << std::endl;
    json << JSONField("num_charts"          , ainfo.nc ,          1)    << "," << std::endl;
    json << JSONField("num_null_charts"     , ainfo.nnc,          1)    << "," << std::endl;
    json << JSONField("mapped_fraction"     , ainfo.mpa,          1)    << "," << std::endl;
    json << JSONField("nfolds"              , ainfo.nfolds,       1)    << "," << std::endl;
    json << JSONField("occupancy"           , ainfo.occupancy,    1)    << "," << std::endl;

    if (cci.valid) {
        json << JSONField("seam_discrepancy_avg"   , (cci.hist.Cnt() == 0) ? 0 : cci.hist.Avg(), 1)      << "," << std::endl;
        json << JSONField("seam_discrepancy_min"   , (cci.hist.Cnt() == 0) ? 0 : cci.hist.MinElem(), 1)  << "," << std::endl;
        json << JSONField("seam_discrepancy_max"   , (cci.hist.Cnt() == 0) ? 0 : cci.hist.MaxElem(), 1)  << "," << std::endl;
        json << JSONField("seam_discrepancy_rms"   , (cci.hist.Cnt() == 0) ? 0 : cci.hist.RMS(), 1)      << "," << std::endl;
        json << JSONField("seam_discrepancy_var"   , (cci.hist.Cnt() == 0) ? 0 : cci.hist.Variance(), 1) << "," << std::endl;
        json << JSONField("seam_discrepancy_stddev", (cci.hist.Cnt() == 0) ? 0 : cci.hist.StandardDeviation(), 1)  << "," << std::endl;
    }

    json << JSONField("ntex"                , (int) ainfo.mipTextureInfo.size(), 1) << "," << std::endl;
    json << JSONField("textures"            , m.textures,                        1) << "," << std::endl;

    for (std::size_t ntex = 0; ntex < ainfo.mipTextureInfo.size(); ntex++) {
        std::stringstream ss;
        ss << "texture_" << ntex;
        json << JSONField(ss.str().c_str(), ainfo.mipTextureInfo[ntex], 1) << "," << std::endl;
    }

    json << JSONField("qcd_perc1" , ainfo.qcHistogram.Percentile(0.01), 1)    << "," << std::endl;
    json << JSONField("qcd_perc10", ainfo.qcHistogram.Percentile(0.10), 1)    << "," << std::endl;
    json << JSONField("qcd_perc20", ainfo.qcHistogram.Percentile(0.20), 1)    << "," << std::endl;
    json << JSONField("qcd_perc30", ainfo.qcHistogram.Percentile(0.30), 1)    << "," << std::endl;
    json << JSONField("qcd_perc40", ainfo.qcHistogram.Percentile(0.40), 1)    << "," << std::endl;
    json << JSONField("qcd_perc50", ainfo.qcHistogram.Percentile(0.50), 1)    << "," << std::endl;
    json << JSONField("qcd_perc60", ainfo.qcHistogram.Percentile(0.60), 1)    << "," << std::endl;
    json << JSONField("qcd_perc70", ainfo.qcHistogram.Percentile(0.70), 1)    << "," << std::endl;
    json << JSONField("qcd_perc80", ainfo.qcHistogram.Percentile(0.80), 1)    << "," << std::endl;
    json << JSONField("qcd_perc90", ainfo.qcHistogram.Percentile(0.90), 1)    << "," << std::endl;
    json << JSONField("qcd_perc99", ainfo.qcHistogram.Percentile(0.99), 1)    << "," << std::endl;
    json << JSONField("qcd_min"   , ainfo.qcHistogram.MinElem(), 1)           << "," << std::endl;
    json << JSONField("qcd_max"   , ainfo.qcHistogram.MaxElem(), 1)           << "," << std::endl;
    json << JSONField("qcd_avg"   , ainfo.qcHistogram.Avg(), 1)               << "," << std::endl;
    json << JSONField("qcd_rms"   , ainfo.qcHistogram.RMS(), 1)               << "," << std::endl;
    json << JSONField("qcd_var"   , ainfo.qcHistogram.Variance(), 1)          << "," << std::endl;
    json << JSONField("qcd_stddev", ainfo.qcHistogram.StandardDeviation(), 1) << "," << std::endl;
    json << JSONField("qcd_discardedArea", ainfo.qcd_areaAboveThreshold, 1)   << "," << std::endl;

    json << JSONField("sf_perc1" , ainfo.sfHistogram.Percentile(0.01), 1)     << "," << std::endl;
    json << JSONField("sf_perc10", ainfo.sfHistogram.Percentile(0.10), 1)     << "," << std::endl;
    json << JSONField("sf_perc20", ainfo.sfHistogram.Percentile(0.20), 1)     << "," << std::endl;
    json << JSONField("sf_perc30", ainfo.sfHistogram.Percentile(0.30), 1)     << "," << std::endl;
    json << JSONField("sf_perc40", ainfo.sfHistogram.Percentile(0.40), 1)     << "," << std::endl;
    json << JSONField("sf_perc50", ainfo.sfHistogram.Percentile(0.50), 1)     << "," << std::endl;
    json << JSONField("sf_perc60", ainfo.sfHistogram.Percentile(0.60), 1)     << "," << std::endl;
    json << JSONField("sf_perc70", ainfo.sfHistogram.Percentile(0.70), 1)     << "," << std::endl;
    json << JSONField("sf_perc80", ainfo.sfHistogram.Percentile(0.80), 1)     << "," << std::endl;
    json << JSONField("sf_perc90", ainfo.sfHistogram.Percentile(0.90), 1)     << "," << std::endl;
    json << JSONField("sf_perc99", ainfo.sfHistogram.Percentile(0.99), 1)     << "," << std::endl;
    json << JSONField("sf_min"   , ainfo.sfHistogram.MinElem(), 1)               << "," << std::endl;
    json << JSONField("sf_max"   , ainfo.sfHistogram.MaxElem(), 1)               << "," << std::endl;
    json << JSONField("sf_avg"   , ainfo.sfHistogram.Avg(), 1)                << "," << std::endl;
    json << JSONField("sf_rms"   , ainfo.sfHistogram.RMS(), 1)                << "," << std::endl;
    json << JSONField("sf_var"   , ainfo.sfHistogram.Variance(), 1)           << "," << std::endl;
    json << JSONField("sf_stddev", ainfo.sfHistogram.StandardDeviation(), 1)  << "," << std::endl;
    json << JSONField("sf_discardedArea", ainfo.sf_areaAboveThreshold, 1)            << std::endl;

    json << "}" << std::endl;

    json.close();

}
