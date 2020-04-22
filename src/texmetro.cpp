#include "gl_utils.h"
#include "mesh.h"
#include "measure.h"
#include "color_consistency.h"

#include <wrap/system/qgetopt.h>

#include <string>
#include <vector>
#include <iostream>
#include <iomanip>

#include <QApplication>

static void ParseTextureSize(const QString& texsize, int *w, int *h);

using namespace vcg;

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

    bool hasTextures = true;

    if (m.texsizes.size() == 0) {
        hasTextures = false;
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

    tri::UpdateTopology<Mesh>::FaceFace(m);

    MeshInfo minfo = ComputeMeshInfo(m);

    // remove duplicate vertices before storing ffadj so that texture seams are
    // always joined in 3D
    if (hasTextures) {
        tri::Clean<Mesh>::RemoveDuplicateVertex(m);
        tri::Allocator<Mesh>::CompactEveryVector(m);
        tri::UpdateTopology<Mesh>::FaceFace(m);
    }

    FaceFaceAdj ffadj = RecordFaceFaceAdjacency(m);

    std::vector<Chart> atlas;
    GenerateAtlas(m, atlas);

    AtlasInfo ainfo = ComputeAtlasInfo(m, atlas);

    ColorConsistencyInfo cci = {};

    if (hasTextures) {
        cci = ComputeColorConsistencyInfo(m, ffadj, argv[1], atlas);
    }

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
    std::cout << "  area_uv " << ainfo.area << std::endl;
    std::cout << "  boundary_len_uv " << ainfo.bnd_len << std::endl;
    std::cout << "  num_charts " << ainfo.nc << std::endl;
    std::cout << "  num_null_charts " << ainfo.nnc << std::endl;
    std::cout << "  mapped_fraction " << ainfo.mpa << std::endl;
    std::cout << "  nfolds " << ainfo.nfolds << std::endl;
    std::cout << "  occupancy " << ainfo.occupancy << std::endl;
    if (cci.valid)
        std::cout << "  seam_discrepancy_avg " << ((cci.hist.Cnt() == 0) ? 0 : cci.hist.Avg()) << std::endl;
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

    WriteJSON(json.toStdString(), m, minfo, ainfo, cci);

    return 0;
}


static void ParseTextureSize(const QString& texsize, int *w, int *h)
{
    bool ok;

    auto it = texsize.begin();
    while (it != texsize.end() && it->isDigit())
        it++;

    *w = texsize.leftRef(it - texsize.begin()).toInt(&ok, 10);
    if (!ok) {
        *w = *h = -1;
        return;
    }

    if (it != texsize.end() && *it == 'x')
        it++;

    *h = texsize.midRef(it - texsize.begin()).toInt(&ok, 10);
    if (!ok) {
        *w = *h = -1;
        return;
    }

    return;
}
