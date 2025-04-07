
#ifndef LINEPLANNING_SPECIALIZEDTD_H
#define LINEPLANNING_SPECIALIZEDTD_H

#include "Graph.h"
#include "TreeDecomposition.h"


namespace TreeDecomposition {

    TreeDecomposition forGrid(const WeightedIndexedGraph<void> &graph);
    TreeDecomposition forRings(const WeightedIndexedGraph<void> &graph);

    class WrongGraphClassError : public std::runtime_error{
    public:
        WrongGraphClassError(std::string msg) : std::runtime_error(msg) {}

    };
}


#endif //LINEPLANNING_SPECIALIZEDTD_H
