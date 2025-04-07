
#ifndef LINEPLANNING_SOLVER2_H
#define LINEPLANNING_SOLVER2_H

#include "../LinePlanning.h"
#include "../TreeDecomposition.h"

namespace Solver{
    LinePlanning::LineConcept solve2(const LinePlanning::Instance& instance, const TreeDecomposition::TreeDecomposition& td);
}



#endif //LINEPLANNING_SOLVER2_H
