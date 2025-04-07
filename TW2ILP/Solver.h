
#ifndef LINEPLANNING_SOLVER_H
#define LINEPLANNING_SOLVER_H

#include "../LinePlanning.h"
#include "../TreeDecomposition.h"
#include <cmath>

namespace Solver{

    struct Options {
        double maxSolveTimeILP = std::numeric_limits<double>::infinity();
        double MIPGap = -1;
        bool allowPaths = true;
        bool allowCycles = false;
        bool enableSpecializedTD = true;
    };

    LinePlanning::LineConcept solve(const LinePlanning::Instance& instance, const TreeDecomposition::TreeDecomposition& td, const Options &options);
}


#endif //LINEPLANNING_SOLVER_H
