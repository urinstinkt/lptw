
#ifndef LINEPLANNING_SOLVER_H
#define LINEPLANNING_SOLVER_H

#include "../LinePlanning.h"
#include "../TreeDecomposition.h"

namespace Solver{

    struct Options {
        bool allowPaths = true;
        bool allowCycles = false;
    };

    LinePlanning::LineConcept solve(const LinePlanning::Instance& instance, const TreeDecomposition::TreeDecomposition& td, const Options &options);
}


#endif //LINEPLANNING_SOLVER_H
