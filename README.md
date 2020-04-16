# texmetro
A tool for evaluating a number of quality metrics on textured 3D models. It computes a number of measures on arbitrary user provided photo-reconstruction models. This tool can be used to assess and compare existing or future alternative photo-reconstruction tools (e.g. starting from the same set of images), as well as to assess the efficacy of existing or future methods aimed to improve the quality of the texturing of 3D models.

The UV-map, geometry, and connectivity of the an input models are analyzed. The statistics in full are collected in a JSON file while only a relevant subset of them is displayed on the standard output, consisting of salient information about the mesh geometry, its parameterization and texture data. A description of the JSON file structure can be found below.

The tool is used to compute the statistics shown in the benchmark  [Real-World Textured Things](http://texturedmesh.isti.cnr.it/index)


A few of the measures are dependent on the texture resolution (in texels). In order to compute these, TexMetro takes as input the actual texture image, or, simply, its dimensions, provided as command-line arguments.
The TexMetro tool is OpenSource and is implemented in C++ using [vcg library](https://github.com/cnr-isti-vclab/vcglib) for mesh processing, Qt (for and image loading and interfacing), and OpenGL (to accelerate texture overlap detecting, which is implemented by rasterization over off-screen buffers)
## Metrics
The tools evaluate the following metrics over a given textured mesh. 

Name  | Description
----------- | -------------
Components	| Number of connected compoents
Charts	    | Number of texture atlas charts
NTextures   |	Number of texture images
 

## JSON format 

