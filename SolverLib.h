
#ifndef LINEPLANNING_SOLVERCOMMON_H
#define LINEPLANNING_SOLVERLIB_H

#include "Graph.h"
#include "LinePlanning.h"
#include "TreeDecomposition.h"
#include "PathPattern.h"

#include <map>
#include <memory>


namespace LinePlanning {
    namespace Solver {

        typedef double cost_t;
        using namespace std;

        template <class T>
        class SumConstraint
        {
            static constexpr unsigned int infinity = -1;

            multiset<T> things;
            unsigned int lower_bound, upper_bound;
            bool falsified = false;

        public:
            SumConstraint(unsigned int lowerBound, unsigned int upperBound = infinity) : lower_bound(lowerBound),
                                                                                         upper_bound(upperBound) {}

            void insertThing(T thing)
            {
                things.insert(thing);
            }

            void substitute(T thing, unsigned int val)
            {
                auto ec = things.erase(thing);
                if (ec != 0)
                {
                    val *= ec;
                    if (lower_bound < val)
                        lower_bound = 0;
                    else
                        lower_bound -= val;
                    if (upper_bound != infinity)
                    {
                        if (upper_bound < val)
                        {
                            upper_bound = 0;
                            falsified = true;
                        }
                        else
                        {
                            upper_bound -= val;
                        }
                    }
                    /*if (!falsified && lower_bound == 0 && (upper_bound == infinity || things.empty()))
                    {
                        satisfied = true;
                    }*/
                }
            }

            bool isFalsified() const
            {
                return falsified;
            }
            bool isSatisfiedByAllZero() const
            {
                return !falsified && lower_bound == 0;
            }
            bool guaranteedSatisfaction() const
            {
                return !falsified && lower_bound == 0 && upper_bound == infinity;
            }
        };

        struct BuildInstruction;

        class PathScheme {
            typedef ReverseSymmetricVector<int> PPT;
            map<PPT, unsigned int> _counts;
        public:
            static constexpr unsigned int defaultValue = 0;

            typedef SumConstraint<PPT> Constraint;

            //static set<PathScheme> allBoundedSchemes(vector<int> vertexSet, unsigned int bound);
            static set<PathScheme> allBoundedSchemes(set<PPT> allowedPatterns, vector<Constraint> constraints);

            void operator+=(const PathScheme& other);

            std::strong_ordering operator<=>(const PathScheme &other) const;

            bool isZero() const;

            void apply(BuildInstruction instruction, set<int> const &currentBag);

            static PathScheme mergeInstruction(PathScheme &ps1, PathScheme &ps2, BuildInstruction instruction, set<int> const &currentBag);

            const map<PPT, unsigned int>& data() const
            {
                return _counts;
            }

            void add(const PathPattern &pp, unsigned int count){
                _counts[PPT{pp.toVector()}] += count;
            }

            void add(const PPT &pp, unsigned int count){
                _counts[pp] += count;
            }

            void assign(const PPT &pp, unsigned int count){
                if (count == 0)
                    _counts.erase(pp);
                else
                    _counts[pp] = count;
            }

            LineConcept toLineConcept(const Instance *instance) const;

            friend ostream& operator<<(ostream& os, PathScheme const& ps);
        };

        struct BuildInstruction {
            enum{CREATE, INSERT, MERGE} mode;
            PathPattern result;
            PathPattern initial;
            PathPattern initial2;
            int to_insert;
            unsigned int insert_index;
            unsigned int frequency;

            cost_t computeCost(const Instance *instance)
            {
                switch (mode){
                    case CREATE:
                    {
                        cost_t cost = 0;
                        for (auto ed : result.edges())
                        {
                            int u1 = ed.first;
                            int u2 = ed.second;
                            cost += frequency*(instance->edgeCost(u1, u2));
                        }
                        cost += frequency*instance->c_fix;
                        return cost;
                    }
                    case INSERT: //subdivide or append
                    {
                        cost_t cost = 0;
                        auto vi = insert_index;
                        if (vi > 0)
                        {
                            int u1 = result.pat[vi-1];
                            int u2 = result.pat[vi];
                            cost += frequency*(instance->edgeCost(u1, u2));
                        }
                        if (vi+1 < result.size())
                        {
                            int u1 = result.pat[vi];
                            int u2 = result.pat[vi+1];
                            cost += frequency*(instance->edgeCost(u1, u2));
                        }
                        if (vi > 0 && vi+1 < result.size()) // it was subdivision -> edge is replaced
                        {
                            int u1 = result.pat[vi-1];
                            int u2 = result.pat[vi+1];
                            cost -= frequency*(instance->edgeCost(u1, u2));
                        }
                        return cost;
                    }
                    case MERGE:
                    {
                        cost_t cost = -(frequency*instance->c_fix);
                        unsigned int i1 = 0,i2 = 0;
                        if (initial.pat[i1] == PathPattern::wildcard)
                            i1++;
                        if (initial2.pat[i2] == PathPattern::wildcard)
                            i2++;
                        //invariant: (initial.pat[i1] == initial2.pat[i2]) != wildcard
                        while (i1+1 < initial.size() && i2+1 < initial2.size())
                        {
                            if (initial.pat[i1+1] == PathPattern::wildcard)
                            {
                                auto u1 = initial2.pat[i2];
                                auto u2 = initial2.pat[i2+1];
                                cost -= frequency*(instance->edgeCost(u1, u2));
                                i1 += 2;
                                i2++;
                            }
                            else if (initial2.pat[i2+1] == PathPattern::wildcard)
                            {
                                auto u1 = initial.pat[i1];
                                auto u2 = initial.pat[i1+1];
                                cost -= frequency*(instance->edgeCost(u1, u2));
                                i1++;
                                i2 += 2;
                            }
                            else
                            {
                                auto u1 = initial.pat[i1];
                                auto u2 = initial.pat[i1+1];
                                cost -= frequency*(instance->edgeCost(u1, u2));
                                i1++;
                                i2++;
                            }
                        }
                        return cost;
                    }
                }
                return 0;
            }
        };

        struct BagChange {
            enum {ADD, REMOVE, NO_CHANGE} mode;
            int vertex;
        };

        struct AugmentedCost {
            cost_t cost;
            vector<BuildInstruction> instructions;
            BagChange bagChange;
            std::shared_ptr<AugmentedCost> child, child2;

            explicit AugmentedCost(cost_t cost) : cost(cost), bagChange{BagChange::NO_CHANGE}, child(nullptr), child2(nullptr)
            {
            }

            explicit AugmentedCost(std::shared_ptr<AugmentedCost> child) : cost(child->cost), bagChange{BagChange::NO_CHANGE}, child(child), child2(nullptr)
            {
            }

            explicit AugmentedCost(std::shared_ptr<AugmentedCost> child, std::shared_ptr<AugmentedCost> child2) : cost(child->cost+child2->cost), bagChange{BagChange::NO_CHANGE}, child(child), child2(child2)
            {
            }

            static AugmentedCost infinity(){
                return AugmentedCost(std::numeric_limits<cost_t>::infinity());
            }

            std::partial_ordering operator<=>(const AugmentedCost &other) const
            {
                return cost <=> other.cost;
            }

            //abuse PathScheme as line concept
            PathScheme reconstructLineConcept(set<int> &bag) const{
                PathScheme ps;
                if (child)
                {
                    ps = child->reconstructLineConcept(bag);
                }
                if (child2)
                {
                    PathScheme ps2 = child2->reconstructLineConcept(bag);
                    PathScheme merged;
                    for (auto ins : instructions)
                    {
                        merged += PathScheme::mergeInstruction(ps, ps2, ins, bag);
                    }
                    ps += ps2;
                    ps += merged;
                    return ps;
                }
                else
                {
                    for (auto ins : instructions)
                    {
                        ps.apply(ins, bag);
                    }
                    switch (bagChange.mode)
                    {
                        case BagChange::ADD:
                            bag.insert(bagChange.vertex);
                            break;
                        case BagChange::REMOVE:
                            bag.erase(bagChange.vertex);
                            break;
                        case BagChange::NO_CHANGE:
                            break;
                    }
                    return ps;
                }
            }

            PathScheme reconstructLineConcept() const{
                set<int> bag;
                return reconstructLineConcept(bag);
            }
        };

    }
}

#endif //LINEPLANNING_SOLVERCOMMON_H
