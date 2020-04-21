#include "color_consistency.h"

#include <QFileInfo>
#include <QDir>
#include <QString>
#include <QImage>

#include <cmath>


typedef std::pair<vcg::Point2d, vcg::Point2d> EdgeUV;

template <typename T> T mix(const T& a, const T& b, double t)
{
    return a * (1 - t) + b * t;
}


static void LoadTextures(const Mesh& m, const char *meshpath, std::vector<QImage>& textures);
static bool IsTextureSeam(int fi, int e, const Mesh& m, const FaceFaceAdj& ffadj, const std::vector<Chart>& atlas);
static void UpdateColorConsistencyInfo(int fi, int e, const Mesh& m, const FaceFaceAdj& ffadj, ColorConsistencyInfo& cci, const std::vector<QImage> &textures);
static std::vector<vcg::Point2d> SampleEdge(const EdgeUV& euv, int nsamples);
static vcg::Point3d SampleTexture(vcg::Point2d p, const QImage& texture);
static vcg::Point3d SampleTexture(int x, int y, const QImage& texture);


using namespace vcg;


ColorConsistencyInfo ComputeColorConsistencyInfo(const Mesh& m, const FaceFaceAdj& ffadj, const char *meshpath, const std::vector<Chart> &atlas)
{
    ColorConsistencyInfo cci;
    cci.Reset();

    std::vector<QImage> textures;
    LoadTextures(m, meshpath, textures);

    assert(m.FN() == (int) m.face.size());

    // todo
    for (int i = 0; i < m.FN(); ++i) {
        for (int j = 0; j < 3; ++j) {
            if (IsTextureSeam(i, j, m, ffadj, atlas)) {
                UpdateColorConsistencyInfo(i, j, m, ffadj, cci, textures);
            }
        }
    }

    cci.valid = true;
    return cci;
}


static void LoadTextures(const Mesh& m, const char *meshpath, std::vector<QImage>& textures)
{
    QFileInfo fi(meshpath);
    fi.makeAbsolute();

    std::string dirname = fi.dir().dirName().toStdString();

    QString wdir = QDir::currentPath();
    QDir::setCurrent(fi.absoluteDir().absolutePath());

    for (const std::string& tname : m.textures) {
        QFileInfo tfi(tname.c_str());
        tfi.makeAbsolute();
        if (!tfi.exists() || !tfi.isReadable()) {
            std::cerr << "Unable to read texture file " << tfi.path().toStdString() << std::endl;
            std::exit(-1);
        }
        textures.push_back(QImage());
        textures.back().load(tfi.absoluteFilePath());
        if (textures.back().isNull()) {
            std::cerr << "Failed to load texture file " << tfi.path().toStdString() << std::endl;
            std::exit(-1);
        }
    }

    QDir::setCurrent(wdir);
}

static bool IsTextureSeam(int fi, int e, const Mesh& m, const FaceFaceAdj& ffadj, const std::vector<Chart> &atlas)
{
    int fj = ffadj.faceIndex[3 * fi + e];
    unsigned ci = m.face[fi].cid;
    unsigned cj = m.face[fj].cid;

    bool border3D = fi == fj;
    bool borderUV = face::IsBorder(m.face[fi], e);
    bool manifold = ffadj.manifold[3 * fi + e];

    return (!border3D) && borderUV && manifold && ((atlas[ci].areaUV > 0) && (atlas[cj].areaUV > 0));
}

static void UpdateColorConsistencyInfo(int fi, int e, const Mesh& m, const FaceFaceAdj& ffadj, ColorConsistencyInfo& cci, const std::vector<QImage>& textures)
{
    const MeshFace& f1 = m.face[fi];
    int e1 = e;

    const MeshFace& f2 = m.face[ffadj.faceIndex[3 * fi + e]];
    int e2 = ffadj.oppositeEdge[3 * fi + e];

    EdgeUV euv1 = std::make_pair(f1.cWT(e1).P(), f1.cWT((e1+1)%3).P());
    EdgeUV euv2 = std::make_pair(f2.cWT(e2).P(), f2.cWT((e2+1)%3).P());

    int tu1 = f1.cWT(0).N();
    int tu2 = f2.cWT(0).N();

    int nsamples = 5 * std::ceil(std::max((euv1.first - euv1.second).Norm(), (euv2.first - euv2.second).Norm()));

    if (nsamples > 0) {
        std::vector<vcg::Point2d> s1 = SampleEdge(euv1, nsamples);
        std::vector<vcg::Point2d> s2 = SampleEdge(euv2, nsamples);

        double discrepancy = 0;
        for (int i = 0; i < nsamples; ++i) {
            vcg::Point3d c1 = SampleTexture(s1[i], textures[tu1]);
            vcg::Point3d c2 = SampleTexture(s2[i], textures[tu2]);
            discrepancy += (c1 - c2).Norm() / (double) nsamples;
        }

        cci.hist.Add(discrepancy, vcg::Distance(m.face[fi].cP0(e), m.face[fi].cP1(e)));
    }
}

static std::vector<vcg::Point2d> SampleEdge(const EdgeUV& euv, int nsamples)
{
    std::vector<vcg::Point2d> samples;
    for (int i = 0; i < nsamples; ++i)
        samples.push_back(mix(euv.first, euv.second, i / (double) nsamples));
    return samples;
}

static vcg::Point3d SampleTexture(vcg::Point2d p, const QImage& texture)
{
    double dummy;

    p.Y() = texture.height() - p.Y();
    p -= vcg::Point2d(0.5, 0.5);

    vcg::Point2d p0 = vcg::Point2d(std::floor(p.X()), std::floor(p.Y()));
    vcg::Point2d p1 = vcg::Point2d(std::floor(p.X() + 1.0), std::floor(p.Y() + 1.0));

    vcg::Point2d w(std::modf(p.X(), &dummy), std::modf(p.Y(), &dummy));

    return mix(
        mix(SampleTexture(int(p0.X()), int(p0.Y()), texture), SampleTexture(int(p1.X()), int(p0.Y()), texture), w.X()),
        mix(SampleTexture(int(p0.X()), int(p1.Y()), texture), SampleTexture(int(p1.X()), int(p1.Y()), texture), w.X()),
        w.Y()
    );
}

static vcg::Point3d SampleTexture(int x, int y, const QImage& texture)
{
    x = (x + texture.width()) % texture.width();
    y = (y + texture.height()) % texture.height();
    QColor c = texture.pixel(x, y);
    return vcg::Point3d(c.red(), c.green(), c.blue());
}
