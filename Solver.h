

#ifndef LINEPLANNING_SOLVER_H
#define LINEPLANNING_SOLVER_H


#include "SolverLib.h"


namespace LinePlanning {
    namespace Solver {

        class LazyCostVector {
        protected:
            typedef shared_ptr<AugmentedCost> ValueType;
            map<PathScheme, ValueType> cost_stored;
            static constexpr cost_t infinity = std::numeric_limits<cost_t>::infinity();
            const Instance* instance;

            virtual pair<ValueType,bool> compute_cost(const PathScheme &pathScheme) = 0;

        public:
            LazyCostVector(const Instance *instance);

            ValueType operator[](const PathScheme &pathScheme)
            {
                auto it = cost_stored.find(pathScheme);
                if (it == cost_stored.end())
                {
                    auto [val, isTrivial] = compute_cost(pathScheme);
                    if (isTrivial) //optimization: don't store trivial values
                    {
                        return val;
                    }
                    else
                    {
                        it = cost_stored.insert(it, make_pair(pathScheme, val));
                    }
                }
                return it->second;
            }
        };

        shared_ptr<AugmentedCost> solve(const Instance& instance, const TreeDecomposition::TreeDecomposition& td);
        shared_ptr<AugmentedCost> solve(const Instance& instance);
    }
}


#endif //LINEPLANNING_SOLVER_H
