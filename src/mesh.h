#ifndef TYPES_H
#define TYPES_H

#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/parametrization/distortion.h>

#include <vector>

class MeshVertex;
class MeshEdge;
class MeshFace;


// default face chart id
const unsigned INVALID_ID = 0xffffffff;

// texture size
struct TextureSize {
    int w;
    int h;
};

// mesh definition
struct MeshUsedTypes : public vcg::UsedTypes<vcg::Use<MeshVertex>::AsVertexType, vcg::Use<MeshEdge>::AsEdgeType, vcg::Use<MeshFace>::AsFaceType>
{
};

class MeshVertex : public vcg::Vertex<MeshUsedTypes, vcg::vertex::Coord3d, vcg::vertex::TexCoord2d, vcg::vertex::Normal3d, vcg::vertex::Color4b, vcg::vertex::BitFlags>
{
};

class MeshFace : public vcg::Face<MeshUsedTypes, vcg::face::VertexRef, vcg::face::FFAdj, vcg::face::WedgeTexCoord2d, vcg::face::Qualityd, vcg::face::BitFlags>
{
public:
    unsigned cid = INVALID_ID;
};

class MeshEdge : public vcg::Edge<MeshUsedTypes, vcg::edge::VertexRef, vcg::edge::BitFlags>
{
};

class Mesh : public vcg::tri::TriMesh<std::vector<MeshVertex>, std::vector<MeshFace> , std::vector<MeshEdge>>
{
public:
    std::string name;
    std::vector<TextureSize> texsizes;
};

// chart definition
struct Chart {
    Mesh& m;
    unsigned cid;
    short ti;
    std::vector<Mesh::FacePointer> fpv;
    double area3D;
    double areaUV;
    double signedAreaUV;
    double boundary3D;
    double boundaryUV;
    int boundaryCount;

    Chart(Mesh& m_, unsigned cid_, short ti_) : m{m_}, cid{cid_}, ti{ti_}, area3D{0}, areaUV{0}, signedAreaUV{0}, boundary3D{0}, boundaryUV{0}, boundaryCount{0} {}
};

// each vector contains 3*fn elements
// data about edge j of face i is stored ad 3*i + j
// assumes each mesh is triangular
struct FaceFaceAdj {
    int fn;
    std::vector<int> faceIndex;
    std::vector<short> oppositeEdge;
    std::vector<bool> manifold;
};

typedef vcg::tri::Distortion<Mesh,true> DistortionWedge;


bool LoadMesh(Mesh &m, const char *filename, int &loadmask);
FaceFaceAdj RecordFaceFaceAdjacency(const Mesh& m);
void GenerateAtlas(Mesh &m, std::vector<Chart>& atlas);

#endif // TYPES_H
