

#include "LinePlanning.h"
#include <unordered_set>
#include <iostream>

using namespace std;

namespace LinePlanning
{

    LineConcept::LineConcept() {

    }

    LineConcept::LineConcept(Line line) : lines({line}) {
    }

    void LineConcept::extendLines(Line::Edge newEdge) {
        for (Line &line : lines)
        {
            line.edges.push_back(newEdge);
        }
    }

    void LineConcept::appendLine(Line line) {
        lines.push_back(line);
    }

    LineConcept LineConcept::splitOffLines(unsigned int totalFrequency) {
        LineConcept split;
        while (totalFrequency > 0)
        {
            Line &back = lines.back();
            if (back.frequency > totalFrequency)
            {
                split.appendLine(Line(totalFrequency, back.edges));
                back.frequency -= totalFrequency;
                break;
            }
            else
            {
                split.appendLine(Line(back));
                totalFrequency -= back.frequency;
                lines.pop_back();
            }
        }
        return split;
    }

    void LineConcept::operator+=(const Line &line) {
        appendLine(line);
    }

    void LineConcept::operator+=(const LineConcept &lineConcept) {
        lines.insert(lines.end(), lineConcept.lines.begin(), lineConcept.lines.end());
    }

    LineConcept::Costs LineConcept::calcCost(const Instance &instance) const{
        Costs costs;
        for (const auto& line : lines) {
            costs.cost_cfix += instance.c_fix*line.frequency;
            for (const auto& edge : line.edges){
                costs.cost_edges += instance.graph.getEdge(edge)->weight.cost*line.frequency;
            }
        }
        costs.costTotal = costs.cost_edges+costs.cost_cfix;
        return costs;
    }

    bool LineConcept::isFeasible(const Instance &instance, bool verbose) const{
        std::unordered_map<InstanceGraph::Index, unsigned int> freq;
        for (auto l : lines) {
            for (auto ei : l.edges) {
                freq[ei] += l.frequency;
            }
        }
        bool ok = true;
        for (auto [ei, e] : instance.graph.edges) {
            if (!(freq[ei] >= e->weight.f_min && freq[ei] <= e->weight.f_max)){
                ok = false;
                if (verbose) {
                    cout << "infeasible: " << ei << " " << freq[ei] << " [" <<e->weight.f_min<< ","<<e->weight.f_max<<"]" << endl;
                }
            }
        }

        return ok;
    }

    Line::Line(unsigned int frequency) : frequency(frequency) {
    }

    Line::Line(unsigned int frequency, LinePlanning::Line::Edge edge) : frequency(frequency), edges({edge})  {

    }

    Line::Line(unsigned int frequency, std::vector<Edge> edges1, std::vector<Edge> edges2) : frequency(frequency){
        edges = edges1;
        edges.insert(edges.end(), edges2.begin(), edges2.end());
    }

    Line::Line(unsigned int frequency, std::vector<Edge> edges) : frequency(frequency), edges(edges){

    }

    Line Line::fromVertexPath(unsigned int frequency, const std::vector<int> &path, const Instance *instance) {
        Line line(frequency);
        for (int i = 0; i+1 < path.size(); i++)
        {
            line.edges.push_back(instance->graph.findEdge(path[i], path[i+1])->index);
        }
        return line;
    }

    std::vector<int> Line::toVertexPath(const Instance *instance) const {
        if (edges.empty())
        {
            return {};
        }
        if (edges.size() == 1)
        {
            auto e = instance->graph.getEdge(edges[0]);
            return {e->leftNode->index, e->rightNode->index};
        }

        std::vector<int> path;
        auto e1 = instance->graph.getEdge(edges[0]);
        auto e2 = instance->graph.getEdge(edges[1]);
        if (e1->leftNode->index == e2->leftNode->index || e1->leftNode->index == e2->rightNode->index)
        {
            path.push_back(e1->rightNode->index);
        }
        else
        {
            path.push_back(e1->leftNode->index);
        }

        for (int j = 0; j < edges.size(); j++)
        {
            auto e = instance->graph.getEdge(edges[j]);
            if (path.back() != e->leftNode->index)
            {
                path.push_back(e->leftNode->index);
            }
            else
            {
                path.push_back(e->rightNode->index);
            }
        }
        return path;
    }

    LineConceptEx::LineConceptEx(LinePlanning::LineConcept lc, double cost) : LineConcept(lc), cost(cost){

    }

    Line merge(unsigned int frequency, Line l1, Line l2) {
        Line line(frequency);
        line.edges = l1.edges;
        line.edges.insert(line.edges.end(), l2.edges.rbegin(), l2.edges.rend());
        return line;
    }

    LineConcept merge(LineConcept lc1, LineConcept lc2) {
        LineConcept merged;
        while (!lc1.lines.empty() && !lc2.lines.empty())
        {
            auto f = std::min(lc1.lines.back().frequency, lc2.lines.back().frequency);
            if (f != 0)
            {
                merged += merge(f,lc1.lines.back(), lc2.lines.back());
            }
            lc1.lines.back().frequency -= f;
            lc2.lines.back().frequency -= f;
            if (lc1.lines.back().frequency == 0)
                lc1.lines.pop_back();
            if (lc2.lines.back().frequency == 0)
                lc2.lines.pop_back();
        }
        return merged;
    }

    unsigned int Instance::getTotalFmax(InstanceGraph::Index vertex) const{
        unsigned int sum = 0;
        auto vit = graph.nodes.find(vertex);
        if (vit != graph.nodes.end())
        {
            for (auto eit : vit->second->incidentEdges)
            {
                sum += eit.second->weight.f_max;
            }
        }
        else
        {
            throw std::runtime_error("error: vertex not found");
        }
        return sum;
    }
}
