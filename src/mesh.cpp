#include "mesh.h"

#include <wrap/io_trimesh/import.h>
#include <vcg/complex/algorithms/attribute_seam.h>

#include <QImageReader>
#include <QDir>
#include <QFileInfo>
#include <QString>


static void CutAlongSeams(Mesh& m);


using namespace vcg;

bool LoadMesh(Mesh &m, const char *filename, int &loadmask)
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

    if (!(loadmask & tri::io::Mask::IOM_WEDGTEXCOORD)) {
        std::cerr << "Model has no texture coordinates" << std::endl;
        return false;
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

FaceFaceAdj RecordFaceFaceAdjacency(const Mesh& m)
{
    FaceFaceAdj ffadj;

    ffadj.fn = m.fn;
    ffadj.faceIndex.reserve(3 * m.fn);
    ffadj.oppositeEdge.reserve(3 * m.fn);
    ffadj.manifold.reserve(3 * m.fn);

    for (auto& f : m.face) {
        for (int i = 0; i < 3; ++i) {
            ffadj.faceIndex.push_back(tri::Index(m, f.cFFp(i)));
            ffadj.oppositeEdge.push_back(f.cFFi(i));
            ffadj.manifold.push_back(face::IsManifold(f, i));
        }
    }

    return ffadj;
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

            assert(cid == atlas.size());

            short ti = f.WT(0).N();
            atlas.push_back(Chart{m, cid, ti});
            while (!s.empty()) {
                auto fp = s.top();
                s.pop();
                fp->cid = cid;
                assert(fp->WT(0).N() == ti);
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
