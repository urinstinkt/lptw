
#ifndef LINEPLANNING_TREESOLVER_H
#define LINEPLANNING_TREESOLVER_H

#include "Graph.h"
#include "LinePlanning.h"
#include <deque>

namespace LinePlanning
{
    namespace TreeSolver {

        typedef double real;

        struct SolutionHint {
            std::vector<unsigned int> minChildEndings;
            std::vector<unsigned int> frequency;

            SolutionHint operator+(const SolutionHint &other) const;
        };

        struct Optimum{
            real cost;
            //SolutionHint sol;
            LineConcept solution;
        };

        struct OptimaTable{

            std::vector<Optimum> optima;

            explicit OptimaTable(unsigned int maximal_k);

            unsigned int getMaximalK() const;
            Optimum& operator[](unsigned int i);
            const Optimum& operator[](unsigned int i) const;
        };

        struct NodeInfo{
            EdgeInfo parentEdge;
            InstanceGraph::Edge *parentEdgeRef;
            OptimaTable ot;

            NodeInfo(LinePlanning::EdgeInfo edgeInfo);
            NodeInfo();
        };

        typedef RootedNodeWeightedTree<NodeInfo> Tree;

        void construct_tables(Tree::Subtree subtree, real c_fix);
        LineConceptEx solve(const Instance &instance);
    }

    /*
    namespace TreeSolverNew {

        typedef double real;

        struct PotentialMatch{
            unsigned int count;
            real cost;
        };

        struct MatchList{
            std::deque<PotentialMatch> matches;

            void increaseCost(real inc)
            {
                for (PotentialMatch &pm : matches)
                {
                    pm.cost += inc;
                }
            }

            void cullExpensiveMatches(real max_cost)
            {
                while (!matches.empty() && matches.back().cost >= max_cost)
                {
                    matches.pop_back();
                }
            }

            void increaseAndCull(real inc, real max_cost)
            {
                increaseCost(inc);
                cullExpensiveMatches(max_cost);
            }

            void limitTotalCount(unsigned int max_count)
            {
                int sum = 0;
                for (auto it = matches.begin(); it != matches.end(); it++)
                {
                    if (sum+it->count > max_count)
                    {
                        it->count = max_count-sum;
                        it++;
                        matches.erase(it, matches.end());
                        break;
                    }
                    else
                    {
                        sum += it->count;
                    }
                }
            }
        };

        struct SolverData {
            MatchList matches;
            real cost;
        };

        struct NodeInfo{
            EdgeInfo parentEdge;
            SolverData data;

            NodeInfo(LinePlanning::EdgeInfo edgeInfo);
            NodeInfo();
        };
        typedef RootedNodeWeightedTree<NodeInfo> Tree;

        SolverData opMerge(const SolverData &t1, const SolverData &t2, real c_fix){
            real cost = t1.cost+t2.cost;
            MatchList ml;

            //base case: 0 exposed match points; match branches as much as possible
            MatchList ml1 = t1.matches;
            MatchList ml2 = t2.matches;
            auto it1 = ml1.matches.begin();
            auto it2 = ml2.matches.begin();
            while (it1 != ml1.matches.end() && it2 != ml2.matches.end())
            {
                if (it1->cost+it2->cost > c_fix)
                    break;
                auto mc = std::min(it1->count, it2->count);
                cost = cost+mc*(it1->cost+it2->cost-c_fix);
                if (it1->count < it2->count)
                {
                    it2->count -= it1->count;
                    it1 = ml1.matches.erase(it1);
                }
                else
                {
                    it1->count -= it2->count;
                    it2 = ml2.matches.erase(it2);
                }
            }

            SolverData sd;
            sd.cost = cost;
            sd.matches = std::move(ml);
            return sd;
        }

        SolverData opEdgeExtension(SolverData &nodeData, const EdgeInfo &edge, real c_fix){
            real cost = nodeData.cost;
            cost += (c_fix+edge.cost)*edge.f_min;   //cost for lines of length 1
            //match new lines downbranch
            unsigned int matchable = edge.f_min;
            MatchList &ml = nodeData.matches;
            while (matchable > 0 && !ml.matches.empty())
            {
                PotentialMatch &pm = ml.matches.front();
                unsigned int mc = std::min(matchable, pm.count);
                matchable -= mc;
                if (pm.cost - c_fix > 0)  //stop if too expensive
                    break;
                cost = cost+mc*(pm.cost - c_fix);
                if (matchable >= pm.count)
                {
                    ml.matches.pop_front();
                }
                else
                {
                    pm.count -= mc;
                }
            }
            //remaining matches get more expensive
            ml.increaseAndCull(edge.cost, c_fix);
            //free match points
            ml.matches.push_front(PotentialMatch{edge.f_min,0});
            //respect f_max
            ml.limitTotalCount(edge.f_max);

            SolverData sd;
            sd.cost = cost;
            sd.matches = std::move(ml);
            return sd;
        }


        SolverData opEdgeExtension(Tree::Subtree tree, real c_fix) {
            const EdgeInfo &edge = tree.weight().parentEdge;
            return opEdgeExtension(tree.weight().data, edge, c_fix);
        }
    }
     */
}


#endif //LINEPLANNING_TREESOLVER_H
