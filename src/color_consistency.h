#ifndef COLOR_CONSISTENCY_H
#define COLOR_CONSISTENCY_H

#include "mesh.h"

#include <vcg/math/histogram.h>

#include <vector>

struct Chart;

struct ColorConsistencyInfo {
    bool valid;
    vcg::Histogram<double> hist;

    void Reset() { valid = false; hist.Clear(); hist.SetRange(0, 1000, 100000); }
};

ColorConsistencyInfo ComputeColorConsistencyInfo(const Mesh& m, const FaceFaceAdj& ffadj, const char *meshpath, const std::vector<Chart>& atlas);

#endif // COLOR_CONSISTENCY_H
