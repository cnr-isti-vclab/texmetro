#include "gl_utils.h"
#include "types.h"
#include "measure.h"

#include <wrap/io_trimesh/import.h>
#include <wrap/system/qgetopt.h>

#include <vcg/complex/algorithms/parametrization/distortion.h>
#include <vcg/complex/algorithms/attribute_seam.h>

#include <string>
#include <vector>
#include <iostream>
#include <iomanip>

#include <QApplication>
#include <QImageReader>
#include <QDir>
#include <QFileInfo>
#include <QString>


static bool LoadMesh(Mesh &m, const char *filename, int &loadmask)
{
    using namespace vcg;

    m.Clear();
    loadmask = 0;

    QFileInfo fi(filename);
    fi.makeAbsolute();

    if (!fi.exists() || !fi.isReadable()) {
        std::cerr << "Unable to read " << filename << std::endl;
        return false;
    }

    m.name = fi.fileName().toStdString();

    QString wd = QDir::currentPath();
    QDir::setCurrent(fi.absoluteDir().absolutePath());

    int r = tri::io::Importer<Mesh>::Open(m, fi.fileName().toStdString().c_str(), loadmask);
    if (r != 0) {
        std::cerr << tri::io::Importer<Mesh>::ErrorMsg(r) << std::endl;
        if (tri::io::Importer<Mesh>::ErrorCritical(r)) {
            return false;
        }
    }

    int ntex = 0;
    for (const string& textureName : m.textures) {
        QFileInfo textureFile(textureName.c_str());
        textureFile.makeAbsolute();
        if (!textureFile.exists() || !textureFile.isReadable()) {
            std::cerr << "Error: Texture file " << textureName.c_str() << " does not exist or is not readable." << std::endl;
            return false;
        }
        QImageReader reader(textureFile.absoluteFilePath());
        QSize sz = reader.size();
        m.texsizes.push_back({sz.width(), sz.height()});
        ntex++;
    }

    // if necessary, recenter uv box
    std::unordered_map<short, vcg::Box2d> uvbb;
    for (auto& f : m.face) {
        for (int i = 0; i < 3; ++i)
            uvbb[f.WT(i).N()].Add(f.WT(i).P());
    }

    for (auto& f : m.face) {
        double offsetU = 0;
        double offsetV = 0;
        vcg::Box2d box = uvbb[f.WT(0).N()];
        vcg::Point2d center = box.Center();
        if (center.X() < 0 || center.X() > 1 || center.Y() < 0 || center.Y() > 1) {
            offsetU = -std::floor(box.min.X()) + 1e-3;
            offsetV = -std::floor(box.min.Y()) + 1e-3;
        }
        for (int i = 0; i < 3; ++i) {
            f.WT(i).U() += offsetU;
            f.WT(i).V() += offsetV;
        }
    }

    assert(ntex == (int) m.texsizes.size());

    QDir::setCurrent(wd);
    return true;
}

using namespace vcg;

static inline void vExt(const Mesh& msrc, const MeshFace& f, int k, const Mesh& mdst, MeshVertex& v)
{
    (void) msrc;
    (void) mdst;
    v.ImportData(*(f.cV(k)));
    v.T() = f.cWT(k);
}

static inline bool vCmp(const Mesh& mdst, const MeshVertex& v1, const MeshVertex& v2)
{
    (void) mdst;
    return v1.T() == v2.T();
}

static void CutAlongSeams(Mesh& m)
{
    int vn = m.vn;
    tri::AttributeSeam::SplitVertex(m, vExt, vCmp);
    if (m.VN() != vn) {
        tri::Allocator<Mesh>::CompactEveryVector(m);
    }
}

void GenerateAtlas(Mesh &m, std::vector<Chart>& atlas)
{
    atlas.clear();
    CutAlongSeams(m);

    tri::UpdateTopology<Mesh>::FaceFace(m);
    tri::UpdateFlags<Mesh>::FaceClearV(m);

    std::stack<Mesh::FacePointer> s;
    unsigned cid = 0;
    int fn = 0;
    for (auto &f : m.face) {
        if (!f.IsV()) {
            f.SetV();
            s.push(&f);
            short sid = f.WT(0).N();
            atlas.push_back(Chart{m, cid, sid});
            while (!s.empty()) {
                auto fp = s.top();
                s.pop();
                fp->cid = cid;
                assert(fp->WT(0).N() == sid);
                atlas.back().fpv.push_back(fp);
                for (int i = 0; i < 3; ++i) {
                    if (!face::IsBorder(*fp, i)) {
                        auto adj = fp->FFp(i);
                        if (!adj->IsV()) {
                            adj->SetV();
                            s.push(adj);
                        }
                    }
                }
            }
            fn += atlas.back().fpv.size();
            cid++;
        }
    }

    assert(fn == m.FN());

    for (auto& chart : atlas) {
        for (auto fp: chart.fpv) {
            chart.area3D += DistortionWedge::Area3D(fp);
            chart.areaUV += std::abs(DistortionWedge::AreaUV(fp));
            chart.signedAreaUV += DistortionWedge::AreaUV(fp);
            for (int i = 0; i < 3; ++i) {
                if (face::IsBorder(*fp, i)) {
                    chart.boundary3D += DistortionWedge::EdgeLenght3D(fp, i);
                    chart.boundaryUV += DistortionWedge::EdgeLenghtUV(fp, i);
                    chart.boundaryCount++;
                }
            }
            assert(chart.cid == fp->cid);
        }
    }
}

void ParseTextureSize(const QString& texsize, int *w, int *h)
{
    bool ok;

    auto it = texsize.begin();
    while (it != texsize.end() && it->isDigit())
        it++;

    *w = texsize.mid(0, it - texsize.begin()).toInt(&ok, 10);
    if (!ok) {
        *w = *h = -1;
        return;
    }

    if (it != texsize.end() && *it == 'x')
        it++;

    *h = texsize.mid(it - texsize.begin()).toInt(&ok, 10);
    if (!ok) {
        *w = *h = -1;
        return;
    }

    return;
}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    QString json = "texmetro.json";
    QString texsize = "";
    int w = -1;
    int h = -1;

    GetOpt opt(argc, argv);
    opt.setHelp("ARGS specifies the 3D model used to compute the measures (the "
                "model is required to be UV mapped). Since some of the measures "
                "computed are sensitive to the texture size, the model should "
                "also include a texture file. For 3D models without textures it "
                "is possible to specify the texture size as an optional argument.");

    opt.allowUnlimitedArguments(true);

    opt.addOption('j', "json", "name of the output json file", &json);
    opt.addOption('t', "texsize", "size of the texture file (eg 1024x512) if the model has none", &texsize);

    opt.parse();

    if (texsize != "") {
        ParseTextureSize(texsize, &w, &h);
        if (w == -1 || h == -1) {
            std::cerr << "Error while parsing the texture size argument " << texsize.toStdString() << std::endl;
            std::exit(-1);
        }
    }

    std::string object;

    QStringList arglist = opt.arguments;
    if (arglist.size() < 1) {
        std::cerr << qPrintable(opt.usage()) << std::endl;
        std::exit(-1);
    } else {
        object = arglist[0].toStdString();
    }

    Mesh m;
    int loadmask;

    std::cout << "Running texmetro on " << object << std::endl;

    if (LoadMesh(m, argv[1], loadmask) == false) {
        std::cerr << "Failed to open mesh " << object << std::endl;
        std::exit(-1);
    }

    if (m.texsizes.size() == 0) {
        if (w > 0 && h > 0) {
            m.texsizes.push_back({w, h});
            for (auto& f : m.face)
                for (int i = 0; i < 3; ++i)
                    f.WT(i).N() = 0;
            m.textures.push_back("N/A");
        } else {
            std::cerr << "Model has no textures and no texsize option was specified, aborting" << std::endl;
            std::exit(-1);
        }
    }

    for (auto& f : m.face) {
        int ti = f.WT(0).N();
        for (int i = 0; i < f.VN(); ++i) {
            f.WT(i).P().X() *= m.texsizes[ti].w;
            f.WT(i).P().Y() *= m.texsizes[ti].h;
        }
    }

    assert(loadmask & tri::io::Mask::IOM_WEDGTEXCOORD);

    MeshInfo minfo = ComputeMeshInfo(m);

    std::vector<Chart> atlas;
    GenerateAtlas(m, atlas);

    AtlasInfo ainfo = ComputeAtlasInfo(m, atlas);

    std::cout << std::endl;
    std::cout << "Mesh geometry" << std::endl;
    std::cout << "  fn " << minfo.fn << std::endl;
    std::cout << "  vn " << minfo.vn << std::endl;
    std::cout << "  en " << minfo.en << std::endl;
    std::cout << "  en_b " << minfo.en_b << std::endl;
    std::cout << "  area " << minfo.area << std::endl;
    std::cout << "  boundary_len " << minfo.bnd_len << std::endl;
    std::cout << "  boundary_loops " << minfo.bl << std::endl;
    std::cout << "  nonmainf_edge " << minfo.nme << std::endl;
    std::cout << "  nonmanif_vert " << minfo.nmv << std::endl;
    std::cout << "  connected_components " << minfo.cc << std::endl;
    std::cout << "  genus " << minfo.g << std::endl;
    std::cout << std::endl;
    std::cout << "Texture atlas" << std::endl;
    std::cout << "  en_uv " << ainfo.en_uv << std::endl;
    std::cout << "  en_uv_b " << ainfo.en_uv_b << std::endl;
    std::cout << "  area_uv " << minfo.area << std::endl;
    std::cout << "  boundary_len_uv " << minfo.bnd_len << std::endl;
    std::cout << "  num_charts " << ainfo.nc << std::endl;
    std::cout << "  num_null_charts " << ainfo.nnc << std::endl;
    std::cout << "  mapped_fraction " << ainfo.mpa << std::endl;
    std::cout << "  nfolds " << ainfo.nfolds << std::endl;
    std::cout << "  occupancy " << ainfo.occupancy << std::endl;
    std::cout << "  ntex " << ainfo.mipTextureInfo.size() << std::endl;

    std::cout << std::endl;
    std::cout << "  Texture name                       Resolution    Occupancy     Overlap       MIP-index" << std::endl;
    std::cout << "  --------------------------------------------------------------------------------------" << std::endl;
    std::cout.setf(std::ios::left);

    for (auto& mti : ainfo.mipTextureInfo) {
        int ti = &mti - &ainfo.mipTextureInfo[0];
        for (unsigned k = 0; k < mti.size(); ++k) {
            if (mti[k].totalFragments > 0) {
                std::string tname = m.textures[ti];
                if (tname.length() > 32) {
                    tname = tname.substr(0, 32);
                    tname[29] = tname[30] = tname[31] = '.';
                }

                std::string tres = std::to_string(m.texsizes[ti].w) + "x" + std::to_string(m.texsizes[ti].h);
                std::string tocc = std::to_string((mti[k].totalFragments - mti[k].lostFragments) / (double) (mti[k].w * mti[k].h));
                std::string tover = std::to_string(mti[k].overwrittenFragments / (double) (mti[k].totalFragments - mti[k].lostFragments));
                unsigned lmip = k;
                for (; lmip < mti.size(); ++lmip)
                    if (mti[lmip].fragmentClashes > 0)
                        break;

                std::cout << "  " << std::setw(32) << tname;
                std::cout << "   " << std::setw(11) << tres;
                std::cout << "   " << std::setw(11) << tocc;
                std::cout << "   " << std::setw(11) << tover;
                std::cout << "   " << std::setw(11) << lmip;

                std::cout << std::endl;

                break;
            }
        }
    }
    std::cout.setf(std::ios::right);

    WriteJSON(json.toStdString(), m, minfo, ainfo);

    return 0;
}
