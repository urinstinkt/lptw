
#ifndef LINEPLANNING_LINEPLANNING_H
#define LINEPLANNING_LINEPLANNING_H

#include "Graph.h"
#include <vector>

namespace LinePlanning
{
    class InfeasibleInstanceError : public std::runtime_error{
    public:
        explicit InfeasibleInstanceError(const std::string& msg) : std::runtime_error(msg) {}

    };

    struct EdgeInfo{
        unsigned int f_min, f_max;
        double cost;
    };
    struct EdgeInfoEx : public EdgeInfo{
        double length;

        EdgeInfoEx(unsigned int f_min, unsigned int f_max)
        {
            this->f_min = f_min;
            this->f_max = f_max;
            this->cost = 0;
            this->length = 0;
        }

        EdgeInfoEx() : EdgeInfoEx(0,0){}
    };

    typedef WeightedIndexedGraph<EdgeInfoEx> InstanceGraph;

    struct Instance{
        InstanceGraph graph;
        double c_fix;

        unsigned int getTotalFmax(InstanceGraph::Index vertex) const;

        double edgeCost(InstanceGraph::Index u, InstanceGraph::Index v) const {
            auto e = graph.findEdge(u, v);
            if (e != nullptr)
                return e->weight.cost;
            else
                return 0;
        }

        double edgeCost(InstanceGraph::Index edgeId) const {
            auto e = graph.getEdge(edgeId);
            if (e != nullptr)
                return e->weight.cost;
            else
                return 0;
        }
    };

    struct Line {
        typedef unsigned int Edge;
        unsigned int frequency;
        std::vector<Edge> edges;

        Line(unsigned int frequency = 0);
        Line(unsigned int frequency, std::vector<Edge> edges);
        Line(unsigned int frequency, Edge edge);
        Line(unsigned int frequency, std::vector<Edge> edges1, std::vector<Edge> edges2);

        static Line fromVertexPath(unsigned int frequency, const std::vector<int> &path, const Instance* instance);
        std::vector<int> toVertexPath(const Instance* instance) const;
    };

    Line merge(unsigned int frequency, Line l1, Line l2);

    struct LineConcept{
        struct Costs {
            double costTotal = 0;
            double cost_cfix = 0;
            double cost_edges = 0;
        };

        std::vector<Line> lines;

        LineConcept();
        LineConcept(Line line);

        void extendLines(Line::Edge newEdge);
        LineConcept splitOffLines(unsigned int totalFrequency);
        void appendLine(Line line);

        void operator+=(const Line &line);
        void operator+=(const LineConcept &lineConcept);

        Costs calcCost(const Instance &instance) const;
        bool isFeasible(const Instance &instance, bool verbose = false) const;
    };

    LineConcept merge(LineConcept lc1, LineConcept lc2);

    struct LineConceptEx : public LineConcept{
        double cost;

        LineConceptEx(LineConcept lc, double cost);
    };
}

#endif //LINEPLANNING_LINEPLANNING_H
