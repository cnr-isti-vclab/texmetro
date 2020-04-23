#ifndef MEASURE_H
#define MEASURE_H

#include <vector>
#include <vcg/math/histogram.h>
#include <Eigen/Core>
#include <Eigen/Core>


class Mesh;
class Chart;
struct AtlasInfo;
struct ColorConsistencyInfo;

constexpr double UV_SCALING_RATIO_MAX_THRESHOLD = 100.0;
constexpr double QUASI_CONFORMAL_DISTORTION_MAX_THRESHOLD = 100.0;

/* Computes the angle between u and v */
template <typename PointType>
double VecAngle(const PointType& u, const PointType& v)
{
   typename PointType::ScalarType nu = u.Norm();
   typename PointType::ScalarType nv = v.Norm();

   double n = (u*nv - v*nu).Norm();
   double d = (u*nv + v*nu).Norm();

   return 2.0 * std::atan(n/d);
}

/* Projects two vectors to the 2d plane preserving angle and lenghts */
template <typename PointType, typename PointTypeOut>
void LocalIsometry(const PointType& v1, const PointType& v2, PointTypeOut& w1, PointTypeOut& w2)
{
    //ensure_condition(v1.Norm() > 0 && v2.Norm() > 0);
    double v1n = v1.Norm();
    double v2n = v2.Norm();
    if (v1n == 0 || v2n == 0) {
        if (v1n == 0) v1n = 1e-6;
        if (v2n == 0) v2n = 1e-6;
    }
    double theta = VecAngle(v1, v2);
    if (!(theta > 0 && theta < M_PI)) {
        if (theta == 0) theta = 1e-3; // push theta to be very small
        else if (theta == M_PI) theta = M_PI - 1e-3; // almost flat
        else theta = M_PI / 2.0; // completely arbitrary value, should never happen
    }
    w1[0] = v1n;
    w1[1] = 0;
    w2[0] = v2n * std::cos(theta);
    w2[1] = v2n * std::sin(theta);
}

/* Computes the 2x2 matrix that maps [x10^T, x20^T] to [u10^T, u20^T] */
template <typename PointType>
Eigen::Matrix2d ComputeTransformationMatrix(const PointType& x10, const PointType& x20, const PointType& u10, const PointType& u20)
{
    Eigen::Matrix2d f;
    Eigen::Matrix2d g;

    f(0, 0) = x10[0];
    f(1, 0) = x10[1];
    f(0, 1) = x20[0];
    f(1, 1) = x20[1];
    g(0, 0) = u10[0];
    g(1, 0) = u10[1];
    g(0, 1) = u20[0];
    g(1, 1) = u20[1];

    return g * f.inverse();
}

struct TexImageInfo {
    int w;  // raster width
    int h;  // raster height
    int totalTexels;
    int totalTexels_bilinear;
    int overwrittenTexels;  // number of texels that were written more than once
    int lostTexels;         // number of texels lost due to overwrites: if texel t has tw>1 writes, then lostTexels += (tw-1)
    int texelClashes;       // clashing texels are texels for which bilinear interpolation hits texels from different charts
    int boundaryTexels;
};

struct MeshInfo {
    int fn;    // faces
    int vn;    // vertices
    int en;    // edges
    int en_b;  // boundary edges
    double area;      // surface area
    double bnd_len;   // boundary length
    double area_normalized;    // surface area scaled to a unitary bounding box (max dim = 1)
    double bnd_len_normalized; // boundary length scaled to a unitary bounding box (max dim = 1)
    int nme;   // non manifold edges
    int nmv;   // non manifold vertices
    int cc;    // connected components
    int bl;    // boundary loops
    int g;     //genus
    int fndup;        // number of duplicate faces
    int fnzero;       // number of zero-area faces
    int fndegen;       // number of degenerate faces
    bool oriented;    // mesh is oriented
    bool orientable;  // mesh is orientable
    int vnunref;      // number of unreferenced vertices
    //int vnt;          // number of t-vertices
};

struct AtlasInfo {
    int en_uv;    // number of edges in UV space
    int en_uv_b;  // number of boundary edges in UV space
    int nc;       // number of charts
    int nnc;      // number of null charts (zero uv area)
    double mpa;   // mapped fraction of surface area
    double area;  // atlas area
    double bnd_len; // boundary length
    int nfolds;   // number of folded faces

    double qcd_areaAboveThreshold;       // fraction of area above the quasi-conformal distortion threshold
    vcg::Histogram<double> qcHistogram;  // histogram of quasi-conformal distoriton

    double sf_areaAboveThreshold;        // fraction of area above the uv scaling ratio threshold
    vcg::Histogram<double> sfHistogram;  // histogram of 3d-to-uv scaling

    std::vector<std::vector<TexImageInfo>> mipTextureInfo;
    double occupancy;  // texture space occupancy
};

std::vector<std::vector<TexImageInfo>> ComputeTexImageInfoAtMipLevels(Mesh& m, const std::vector<Chart>& atlas);

MeshInfo ComputeMeshInfo(Mesh& m);
AtlasInfo ComputeAtlasInfo(Mesh& m, const std::vector<Chart>& atlas);

void WriteJSON(const std::string& filename, const Mesh& m, const MeshInfo& minfo, AtlasInfo& ainfo, ColorConsistencyInfo &cci);

#endif // MEASURE_H

