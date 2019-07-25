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
    short sid;
    std::vector<Mesh::FacePointer> fpv;
    double area3D;
    double areaUV;
    double boundary3D;
    double boundaryUV;
    int boundaryCount;

    Chart(Mesh& m_, unsigned cid_, short sid_) : m{m_}, cid{cid_}, sid{sid_}, area3D{0}, areaUV{0}, boundary3D{0}, boundaryUV{0}, boundaryCount{0} {}
};



typedef vcg::tri::Distortion<Mesh,true> DistortionWedge;

#endif // TYPES_H
