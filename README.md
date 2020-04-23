# TexMetro

A tool for evaluating a number of quality metrics on textured 3D models. It computes a number of measures on arbitrary user provided photo-reconstruction models. This tool can be used to assess and compare existing or future alternative photo-reconstruction tools (e.g. starting from the same set of images), as well as to assess the efficacy of existing or future methods aimed to improve the quality of the texturing of 3D models.

The UV-map, geometry, and connectivity of the an input models are analyzed. The statistics in full are collected in a JSON file while only a relevant subset of them is displayed on the standard output, consisting of salient information about the mesh geometry, its parameterization and texture data. A description of the JSON file structure can be found below.

The tool is used to compute the statistics shown in the benchmark  [Real-World Textured Things](http://texturedmesh.isti.cnr.it/index)


A few of the measures are dependent on the texture resolution (in texels). In order to compute these, TexMetro takes as input the actual texture image, or, simply, its dimensions, provided as command-line arguments.
The TexMetro tool is OpenSource and is implemented in C++ using [vcg library](https://github.com/cnr-isti-vclab/vcglib) for mesh processing, Qt (for and image loading and interfacing), and OpenGL (to accelerate texture overlap detecting, which is implemented by rasterization over off-screen buffers)

## Usage

Texmetro supports OBJ, PLY and FBX 3D models. To compute the metrics for a given model simply run

    texmetro path/to/model.obj

By default, texmetro creates a JSON output file named texmetro.json. You can use the -j --json option to specify a custom name:

    texmetro path/to/model.obj --json model_data.json

If your model has texure coordinates but no actual texture data, you can use the -t --texsize option to specify a virtual texture size that will be used to compute the texture-related metrics (with the exception of the seam color discrepancy, for obvious reasons):

    texmetro path/to/model_without_texture.obj -t 2048x1024

## Metrics and JSON format 

The JSON file contains a single object that encoding a dictionary of all the metrics computed by TexMetro. The structure of the object is for the most part straightforward.

### General mesh-related metrics

Entry | Description
------|------------
`mesh`                    | The name of the mesh file
`fn`                      | Number of faces
`vn`                      | Number of vertices
`en`                      | Number of edges
`en_b`                    | Number of boundary edges
`area`                    | Surface area
`boundary_len`            | Boundary length
`area_normalized`         | Normalized surface area (max bounding box side equal to 1)
`boundary_len_normalized` | Normalized boundary length (max bounding box side equal to 1)
`boundary_loops`       | Number of boundary loops
`nonmanif_edge`        | Number of non-manifold edges
`nonmanif_vert`        | Number of non-manifold vertices
`connected_components` | Number of connected components
`genus`                | Surface genus
`fndup`                | Number of duplicated faces
`fnzero`               | Number of zero-area faces
`fndegen`              | Number of degenerate faces (non 3-simplexes)
`vnunref`              | Number of unreferenced vertices
`oriented`             | Mesh is coheriently oriented (bool)
`orientable`           | Mesh is orientable (bool)

### UV and texture related metrics

Entry | Description
------|------------
`num_charts`      | Number of UV charts
`en_uv`           | Number of UV edges
`en_uv_b`         | Number of UV boundary edges
`area_uv`         | UV area
`boundary_len_uv` | UV boundary length
`num_null_charts` | Number of charts with zero UV area
`mapped_fraction` | Fraction of the 3D surface that is UV mapped
`nfolds`          | Number of inverted UV triangles (an inverted triangle is a triangle whose signed area does not agree with the signed area of its chart)
`ntex`      | Number of texture files
`textures`  | List of texture file names
`texture_N` | List of raster stats for texture N at various MIP levels (see _Raster statistics_ below for details)
`occupancy` | Fraction of the available texture area that is mapped to the 3D surface
`seam_discrepancy_avg`    |  Average seam color discrepancy (weighted by 3D edge length)
`seam_discrepancy_min`    |  Minimum seam color discrepancy
`seam_discrepancy_max`    |  Maximum seam color discrepancy
`seam_discrepancy_stddev` |  Seam color discrepancy standard deviation
`qcd_perc1`         |  Quasi-conformal distortion 1st percentile 
`qcd_perc10`        |  Quasi-conformal distortion 10th percentile 
`qcd_perc20`        |  Quasi-conformal distortion 20th percentile 
`qcd_perc30`        |  Quasi-conformal distortion 30th percentile 
`qcd_perc40`        |  Quasi-conformal distortion 40th percentile 
`qcd_perc50`        |  Quasi-conformal distortion 50th percentile 
`qcd_perc60`        |  Quasi-conformal distortion 60th percentile 
`qcd_perc70`        |  Quasi-conformal distortion 70th percentile 
`qcd_perc80`        |  Quasi-conformal distortion 80th percentile 
`qcd_perc90`        |  Quasi-conformal distortion 90th percentile 
`qcd_perc99`        |  Quasi-conformal distortion 99th percentile 
`qcd_min`           |  Minimum Quasi-conformal distortion
`qcd_max`           |  Maximum Quasi-conformal distortion
`qcd_avg`           |  Average Quasi-conformal distortion (weighted by 3D area)
`qcd_stddev`        |  Quasi-conformal distortion standard deviation
`qcd_discardedArea` |  3D area of outlier elements that were discarded when computing qcd statistics
`sf_perc1`         |  UV-scaling 1st percentile 
`sf_perc10`        |  UV-scaling 10th percentile 
`sf_perc20`        |  UV-scaling 20th percentile 
`sf_perc30`        |  UV-scaling 30th percentile 
`sf_perc40`        |  UV-scaling 40th percentile 
`sf_perc50`        |  UV-scaling 50th percentile 
`sf_perc60`        |  UV-scaling 60th percentile 
`sf_perc70`        |  UV-scaling 70th percentile 
`sf_perc80`        |  UV-scaling 80th percentile 
`sf_perc90`        |  UV-scaling 90th percentile 
`sf_perc99`        |  UV-scaling 99th percentile 
`sf_min`           |  Minimum UV-scaling
`sf_max`           |  Maximum UV-scaling
`sf_avg`           |  Average UV-scaling (weighted by 3D area)
`sf_stddev`        |  UV-scaling standard deviation
`sf_discardedArea` |  3D area of outlier elements that were discarded when computing sf statistics

### Raster statistics

For each texture image, TexMetro computes the following statistics at multiple MIP levels and groups them in a list sorted by increasing MIP level

Entry | Description
------|------------
`rw`                   |  Raster width at the current MIP level
`rh`                   |  Raster height at the current MIP level
`totalTexels`          |  Number of mapped texels in the texture
`totalTexels_bilinear` |  Number of texels involved in bilinear interpolations
`overwrittenTexels`    |  Number of texels that are mapped to at least two different triangles
`lostTexels`           |  Number of texels that are lost due to overwrites
`texelClashes`         |  Number of texels that can interfere in the bilinear interpolation of texels from different charts
`boundaryTexels`       |  Number of boundary texels

