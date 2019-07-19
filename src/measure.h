#ifndef MEASURE_H
#define MEASURE_H

#include <vector>
#include <vcg/math/histogram.h>

class Mesh;
class Chart;
struct AtlasInfo;

struct TexImageInfo {
    int w; // raster width
    int h; // raster height
    int totalFragments;
    int totalFragments_bilinear;
    int overwrittenFragments; // number of fragments that were written more than once
    int lostFragments; // number of fragments lost due to overwrites: if fragment f has fw>1 writes, than lostFragmens += (fw-1)
    int fragmentClashes; // clashing fragments are fragments for which bilinear interpolation hits fragments from different patches
    int boundaryFragments;
};

struct MeshInfo {
    int fn; // faces
    int vn; // vertices
    int en;  // edges
    int en_b; // boundary edges
    int nme; // non manifold edges
    int nmv; // non manifold vertices
    int cc; // connected components
    int bl; // boundary loops
    int g; //genus
};

struct AtlasInfo {
    int en_seam; // seam edges
    int nc; // number of charts
    int nnc; // number of null charts (zero uv area)
    double mpa; // mapped fraction of surface area
    int nfolds; // number of folded faces
    vcg::Distribution<double> sfDistrib; // distribution of 3d-to-uv scaling
    vcg::Histogram<double> qcHist; // histogram of quasi-conformal distoriton
    std::vector<std::vector<TexImageInfo>> mipTextureInfo;
    double occupancy; // texture space occupancy
};

std::vector<std::vector<TexImageInfo>> ComputeTexImageInfoAtMipLevels(Mesh& m, const std::vector<Chart>& atlas);

MeshInfo ComputeMeshInfo(Mesh& m);
AtlasInfo ComputeAtlasInfo(Mesh& m, const std::vector<Chart>& atlas, const MeshInfo& minfo);

void WriteJSON(const std::string& filename, const Mesh& m, const MeshInfo& minfo, AtlasInfo& ainfo);

#endif // MEASURE_H

