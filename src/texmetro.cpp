#include "gl_utils.h"
#include "types.h"
#include "measure.h"

#include <wrap/io_trimesh/import.h>

#include <vcg/complex/algorithms/parametrization/distortion.h>
#include <vcg/complex/algorithms/attribute_seam.h>

#include <string>
#include <vector>
#include <iostream>
#include <iomanip>

#include <QCoreApplication>
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

    m.name = fi.absoluteFilePath().toStdString();

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

    // make sure the uv coords are anchored at (0,0)
    std::unordered_map<short, vcg::Box2d> uvbb;
    for (auto& f : m.face) {
        for (int i = 0; i < 3; ++i)
            uvbb[f.WT(i).N()].Add(f.WT(i).P());
    }
    for (auto& f : m.face) {
        vcg::Point2d offset = uvbb[f.WT(0).N()].min;
        for (int i = 0; i < 3; ++i) {
            f.WT(i).U() -= std::floor(offset.X());
            f.WT(i).V() -= std::floor(offset.Y());
        }
    }

    assert(ntex == (int) m.texsizes.size());

    for (auto& f : m.face) {
        int ti = f.WT(0).N();
        for (int i = 0; i < f.VN(); ++i) {
            f.WT(i).P().X() *= (ti < ntex) ? m.texsizes[ti].w : 1;
            f.WT(i).P().Y() *= (ti < ntex) ? m.texsizes[ti].h : 1;
        }
    }

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
            chart.areaUV += DistortionWedge::AreaUV(fp);
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

int main(int argc, char *argv[])
{
    // Make sure the executable directory is added to Qt's library path
    {
        QFileInfo executableInfo(argv[0]);
        QCoreApplication::addLibraryPath(executableInfo.dir().absolutePath());
    }
    
    GLInit();

    Mesh m;
    int loadmask;

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " obj" << std::endl;
        std::exit(-1);
    }
    
    std::cout << "Running texmetro on " << argv[1] << std::endl;

    if (LoadMesh(m, argv[1], loadmask) == false) {
        std::cerr << "Failed to open mesh " << argv[1] << std::endl;
        std::exit(-1);
    }

    assert(loadmask & tri::io::Mask::IOM_WEDGTEXCOORD);

    MeshInfo minfo = ComputeMeshInfo(m);

    std::vector<Chart> atlas;
    GenerateAtlas(m, atlas);

    AtlasInfo ainfo = ComputeAtlasInfo(m, atlas, minfo);

    std::cout << std::endl;
    std::cout << "Mesh geometry" << std::endl;
    std::cout << "  fn " << minfo.fn << std::endl;
    std::cout << "  vn " << minfo.vn << std::endl;
    std::cout << "  en " << minfo.en << std::endl;
    std::cout << "  en_boundary " << minfo.en_b << std::endl;
    std::cout << "  en_seam " << ainfo.en_seam << std::endl;
    std::cout << "  nme " << minfo.nme << std::endl;
    std::cout << "  nmv " << minfo.nmv << std::endl;
    std::cout << "  cc " << minfo.cc << std::endl;
    std::cout << "  bl " << minfo.bl << std::endl;
    std::cout << "  g " << minfo.g << std::endl;
    std::cout << std::endl;
    std::cout << "Texture atlas" << std::endl;
    std::cout << "  nc " << ainfo.nc << std::endl;
    std::cout << "  nnc " << ainfo.nnc << std::endl;
    std::cout << "  mapped_area " << ainfo.mpa << std::endl;
    std::cout << "  nfolds " << ainfo.nfolds << std::endl;
    std::cout << "  texture_occupancy " << ainfo.occupancy << std::endl;
    std::cout << "  ntex" << ainfo.mipTextureInfo.size() << std::endl;

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

    WriteJSON("out", m, minfo, ainfo);

    GLTerminate();

    return 0;
}
