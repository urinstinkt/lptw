
#include "TreeSolver.h"
#include <map>
#include <set>

using namespace std;

namespace LinePlanning
{
    namespace TreeSolver {

        typedef Tree::Subtree Subtree;

        OptimaTable leafTable()
        {
            OptimaTable ot(0);
            ot[0] = {0,{}};
            return ot;
        }

        OptimaTable introduceParentNode(const OptimaTable &childTable, const EdgeInfo &edge, unsigned int edgeIndex, real c_fix)
        {
            OptimaTable parentIntroductionTable(edge.f_max);
            for (unsigned int k = 0; k <= parentIntroductionTable.getMaximalK(); k++)
            {
                auto lines_ending_in_v = std::max(edge.f_min, k);
                real t1 = lines_ending_in_v*(c_fix+edge.cost);
                real t2 = std::numeric_limits<real>::max();
                unsigned int best_l;
                real savings = 0;
                for (unsigned int l = 0; l <= lines_ending_in_v && l <= childTable.getMaximalK(); l++)
                {
                    auto x = childTable[l].cost - savings;
                    if (x < t2)
                    {
                        t2 = x;
                        best_l = l;
                    }
                    savings += c_fix;
                }
                parentIntroductionTable[k].cost = t1+t2;
                LineConcept solution = childTable[best_l].solution;
                LineConcept split = solution.splitOffLines(best_l);
                split.extendLines(edgeIndex);
                solution += split;
                unsigned int newlineFreq = lines_ending_in_v-best_l;
                if (newlineFreq > 0)
                    solution += Line(newlineFreq, edgeIndex);
                parentIntroductionTable[k].solution = solution;
            }
            return parentIntroductionTable;
        }

        OptimaTable mergeOperation(const OptimaTable &ot1, const OptimaTable &ot2, real c_fix)
        {
            OptimaTable mergedTable(ot1.getMaximalK()+ot2.getMaximalK());
            for (unsigned int k = 0; k <= mergedTable.getMaximalK(); k++)
            {
                real min_cost = std::numeric_limits<real>::max();
                // minimum admissible k1: k-ot2.getMaximalK() = k1
                // maximum admissible k1: k1 = ot1.getMaximalK()
                // no possible k1 iff: k > ot1.getMaximalK()+ot2.getMaximalK()
                unsigned int best_l;
                unsigned int best_k1;
                for (unsigned int k1 = k>ot2.getMaximalK() ? k-ot2.getMaximalK() : 0; k1 <= k && k1 <= ot1.getMaximalK(); k1++)
                {
                    unsigned int k2 = k-k1;
                    real savings = 0;
                    for (unsigned int l = 0; l+k1 <= ot1.getMaximalK() && l+k2 <= ot2.getMaximalK(); l++)
                    {
                        auto x = ot1[k1+l].cost+ot2[k2+l].cost-savings;
                        if (x < min_cost)
                        {
                            min_cost = x;
                            best_l = l;
                            best_k1 = k1;
                        }
                        savings += c_fix;
                    }
                }
                mergedTable[k].cost = min_cost;
                unsigned int best_k2 = k-best_k1;
                LineConcept sol1 = ot1[best_k1+best_l].solution;
                LineConcept sol2 = ot2[best_k2+best_l].solution;
                LineConcept ms1 = sol1.splitOffLines(best_l);
                LineConcept ms2 = sol2.splitOffLines(best_l);
                LineConcept split1 = sol1.splitOffLines(best_k1);
                ms1 = merge(ms1, ms2);
                LineConcept solution = sol1;
                solution += ms1;
                solution += sol2;
                solution += split1;
                mergedTable[k].solution = solution;
            }
            return mergedTable;
        }

        void construct_tables(Subtree subtree, real c_fix)
        {
            if (subtree.isLeaf())
            {
                subtree.weight().ot = leafTable();
            }
            else
            {
                OptimaTable mergedTable = leafTable();
                for (Subtree child : subtree)
                {
                    construct_tables(child, c_fix);
                    const OptimaTable &childTable = child.weight().ot;
                    const EdgeInfo &edge = child.weight().parentEdge;
                    OptimaTable parentIntroductionTable = introduceParentNode(childTable, edge, child.weight().parentEdgeRef->index, c_fix);
                    mergedTable = mergeOperation(mergedTable, parentIntroductionTable, c_fix);
                }
                subtree.weight().ot = mergedTable;
            }
        }

        /*
        struct DynamicLineConcept{
            std::vector<Line*> lines;

            DynamicLineConcept()
            {}
            DynamicLineConcept(Line *line) : lines({line})
            {
            }

            void deleteAll()
            {
                for (Line *l : lines)
                {
                    delete l;
                }
                lines.clear();
            }

            LineConcept collect()
            {
                map<set<Line::Edge>, unsigned int> fq;
                for (Line *l : lines)
                {
                    set<Line::Edge> s(l->edges.begin(), l->edges.end());
                    fq[s] += l->frequency;
                }
                LineConcept lc;
                for (auto p : fq)
                {
                    if (p.second > 0)
                    {
                        Line line(p.second);
                        line.edges = {p.first.begin(), p.first.end()};
                        lc.lines.push_back(line);
                    }
                }
                return lc;
            }
        };


        template <class Val>
        void findTwoMaxima(const vector<Val> &vec, unsigned int &mi1, unsigned int &mi2)
        {
            Val v1 = vec[0];
            Val v2 = vec[1];
            mi1 = 0;
            mi2 = 1;
            for (unsigned int i = 2; i < vec.size(); i++)
            {
                if (vec[i] > v1)
                {
                    v1 = vec[i];
                    mi1 = i;
                }
                else if (vec[i] > v2)
                {
                    v2 = vec[i];
                    mi2 = i;
                }
            }
        }

        void construct_line_concept(Tree::Subtree subtree, unsigned int targetLineEndings, DynamicLineConcept openLines, unsigned int totalOpenFrequency, DynamicLineConcept &collection)
        {
            const Optimum &optimum = subtree.weight().ot[targetLineEndings];
            vector<DynamicLineConcept> toMergeLCs;
            vector<unsigned int> toMergeCounts;
            vector<DynamicLineConcept> openDownLCs;
            if (subtree.isLeaf())
            {

            }
            else
            {
                unsigned int child_index = 0;
                for (Tree::Subtree child : subtree)
                {
                    auto frequency = optimum.sol.frequency[child_index];
                    auto edge = child.weight().parentEdgeRef->index;
                    Line *line = new Line(frequency, edge);
                    collection.lines.push_back(line);
                    openDownLCs.push_back(line);
                    toMergeLCs.push_back(line);
                    toMergeCounts.push_back(frequency);
                    child_index++;
                }
                toMergeLCs.push_back(openLines);
                toMergeCounts.push_back(totalOpenFrequency);

                while (true)
                {
                    unsigned int mi1,mi2;
                    findTwoMaxima(toMergeCounts, mi1, mi2);
                    unsigned int mc = std::min(toMergeCounts[mi1], toMergeCounts[mi2]);
                    if (mc == 0)
                        break;
                    Line *l1 = toMergeLCs[mi1].lines.back();
                    Line *l2 = toMergeLCs[mi2].lines.back();
                    mc = std::min(1u, std::min(l1->frequency, l2->frequency));
                    toMergeCounts[mi1] -= mc;
                    toMergeCounts[mi2] -= mc;
                    l1->frequency -= mc;
                    l2->frequency -= mc;
                    Line *mergedLine = new Line(mc, l1->edges, l2->edges);
                    collection.lines.push_back(mergedLine);
                    if (mi1 < openDownLCs.size())
                        openDownLCs[mi1].lines.push_back(mergedLine);
                    if (mi2 < openDownLCs.size())
                        openDownLCs[mi2].lines.push_back(mergedLine);
                    if (l1->frequency == 0)
                        toMergeLCs[mi1].lines.pop_back();
                    if (l2->frequency == 0)
                        toMergeLCs[mi2].lines.pop_back();
                }

                child_index = 0;
                for (Tree::Subtree child : subtree)
                {
                    auto ec = optimum.sol.minChildEndings[child_index];
                    auto frequency = optimum.sol.frequency[child_index];
                    construct_line_concept(child, ec, openDownLCs[child_index], frequency, collection);
                    child_index++;
                }
            }
        }

        LineConcept construct_line_concept(Tree::Subtree tree)
        {
            DynamicLineConcept collection;
            construct_line_concept(tree, 0, DynamicLineConcept{}, 0, collection);
            LineConcept lc = collection.collect();
            collection.deleteAll();
            return lc;
        }*/

        LineConceptEx solve(const Instance &instance) {
            auto wc = [](InstanceGraph::Edge* e){
                TreeSolver::NodeInfo ni(e->weight);
                ni.parentEdgeRef = e;
                return ni;
            };
            Tree tree = Tree::fromGraph(instance.graph, wc);
            construct_tables(tree.getRoot(), instance.c_fix);
            LineConcept lc = tree.getRoot().weight().ot[0].solution;
            double cost = tree.getRoot().weight().ot[0].cost;
            return LineConceptEx{lc, cost};
        }

        OptimaTable::OptimaTable(unsigned int maximal_k) : optima(maximal_k+1)
        {}

        unsigned int OptimaTable::getMaximalK() const {
            return optima.size()-1;
        }

        Optimum &OptimaTable::operator[](unsigned int i) {
            return optima[i];
        }

        const Optimum &OptimaTable::operator[](unsigned int i) const {
            return optima[i];
        }

        NodeInfo::NodeInfo(LinePlanning::EdgeInfo edgeInfo) : parentEdge(edgeInfo), ot(0) {}

        NodeInfo::NodeInfo() : NodeInfo(LinePlanning::EdgeInfo{}){
        }

        SolutionHint SolutionHint::operator+(const SolutionHint &other) const {
            SolutionHint sh;
            sh.minChildEndings = this->minChildEndings;
            sh.minChildEndings.insert(sh.minChildEndings.end(), other.minChildEndings.begin(), other.minChildEndings.end());
            sh.frequency = this->frequency;
            sh.frequency.insert(sh.frequency.end(), other.frequency.begin(), other.frequency.end());
            return sh;
        }
    };
}