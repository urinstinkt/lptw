
#ifndef LINEPLANNING_TREEDECOMPOSITION_H
#define LINEPLANNING_TREEDECOMPOSITION_H

#include "Graph.h"
#include <iostream>
#include <set>
#include <algorithm>
#include <filesystem>

namespace TreeDecomposition
{
    using std::vector;
    using std::set;

    extern std::string TD_App_Classpath;

    typedef unsigned int Vertex;

    struct Edge
    {
        Vertex u,v;
    };
    struct EdgeListGraph
    {
        unsigned int vertexCount, edgeCount;
        vector<Edge> edges;

        EdgeListGraph(unsigned int vertexCount = 0) : vertexCount(vertexCount), edgeCount(0) {}

        void addEdge(Edge e) {
            edges.push_back(e);
            edgeCount++;
        }

        void print(std::ostream &ostream) const;

        void indexNormalization(std::unordered_map<Vertex, Vertex> &renaming, std::unordered_map<Vertex, Vertex> &renamingInverse, Vertex startingIndex = 1) const {
            renaming.clear();
            renamingInverse.clear();
            std::set<Vertex> allVertices;
            for (const auto &e : edges){
                for (auto v : {e.u, e.v}) {
                    allVertices.insert(v);
                }
            }
            for (auto v : allVertices){
                renaming[v] = startingIndex;
                renamingInverse[startingIndex] = v;
                startingIndex++;
            }
        }

        void renameVertices(const std::unordered_map<Vertex, Vertex> &renaming) {
            for (Edge &e : edges) {
                e.u = renaming.at(e.u);
                e.v = renaming.at(e.v);
            }
        }
    };


    struct Bag {
        set<Vertex> vertices;
        vector<Bag*> children;
        Bag* parent = nullptr;

        template <class F>
        auto niceVisit(F leafVisitorConstructor) const
        {
            using Visitor = decltype(leafVisitorConstructor());
            if (children.empty())
            {
                Visitor visitor = leafVisitorConstructor();
                for (auto v : vertices)
                {
                    visitor.introduce(v);
                }
                return visitor;
            }
            else
            {
                //optimization: vertices that are not in any child can be introduced later
                set<Vertex> introduce_later = vertices;
                for (auto child : children)
                {
                    set<Vertex> tmp;
                    std::set_difference(introduce_later.begin(), introduce_later.end(), child->vertices.begin(), child->vertices.end(), std::inserter(tmp, tmp.begin()));
                    swap(tmp, introduce_later);
                }
/*#ifndef NDEBUG
                std::cout << "introduce_later: {";
                for (auto i : introduce_later)
                    std::cout << i << " ";
                std::cout << "}" << std::endl;
#endif*/
                auto for_child = [&](Bag* child){
                    vector<Vertex> to_introduce;
                    std::set_difference(vertices.begin(), vertices.end(), child->vertices.begin(), child->vertices.end(), std::back_inserter(to_introduce));
                    vector<Vertex> tmp;
                    std::set_difference(to_introduce.begin(), to_introduce.end(), introduce_later.begin(), introduce_later.end(), std::back_inserter(tmp));
                    swap(tmp, to_introduce);

                    vector<Vertex> to_forget;
                    std::set_difference(child->vertices.begin(), child->vertices.end(), vertices.begin(), vertices.end(), std::back_inserter(to_forget));
                    Visitor visitor = child->niceVisit(leafVisitorConstructor);
                    for (auto v : to_forget)
                    {
                        visitor.forget(v);
                    }
                    for (auto v : to_introduce)
                    {
                        visitor.introduce(v);
                    }
                    return visitor;
                };
                auto it = children.begin();
                Visitor mergedV = for_child(*it);
                it++;
                while (it != children.end())
                {
                    mergedV.merge(for_child(*it));
                    it++;
                }
                for (auto v : introduce_later)
                {
                    mergedV.introduce(v);
                }
                return mergedV;
            }
        }
    };


    class TreeDecomposition;
    TreeDecomposition parse(std::istream &istream);

    class TreeDecomposition{
    protected:
        unsigned int treewidth;
        vector<Bag> bags;

    public:
        TreeDecomposition(unsigned int bagCount);
        TreeDecomposition(const TreeDecomposition&) = delete;
        TreeDecomposition(TreeDecomposition&& other);

        void operator=(TreeDecomposition&& other);

        unsigned int getLargestBagSize() const;
        unsigned int getTreeWidth() const;

        bool isPath();

        void toGraphViz(std::ostream& os) const;

        friend TreeDecomposition parse(std::istream &istream);

        const Bag* root() const;

        template <class F>
        auto niceVisit(F leafVisitorConstructor, bool rootForgetsAll = true) const
        {
            if (rootForgetsAll)
            {
                Bag newRoot;
                newRoot.children = {(Bag*)root()};
                return newRoot.niceVisit(leafVisitorConstructor);
            }
            else
            {
                return root()->niceVisit(leafVisitorConstructor);
            }
        }

        void renameVertices(const std::unordered_map<Vertex, Vertex> &renaming);


        void write(std::ostream &stream) const;

        template <class BagEnumerable>
        static TreeDecomposition fromPathDecomposition(const BagEnumerable &bags){
            TreeDecomposition td(bags.size());
            td.treewidth = 0;
            unsigned int i = 0;
            for (auto bag : bags) {
                td.bags[i].vertices = set<Vertex>(bag.begin(), bag.end());
                td.treewidth = std::max(td.treewidth, (unsigned int)td.bags[i].vertices.size()-1);
                if (i > 0){
                    td.bags[i].parent = &td.bags[i-1];
                    td.bags[i-1].children = {&td.bags[i]};
                }
                i++;
            }
            return td;
        }
    };

    template <class T>
    EdgeListGraph convert(const WeightedIndexedGraph<T> &graph)
    {
        EdgeListGraph elg;
        elg.vertexCount = graph.nodeCount();
        elg.edgeCount = graph.edges.size();
        for (auto edge : graph.edges)
        {
            auto i1 = edge.second->leftNode->index;
            auto i2 = edge.second->rightNode->index;
            Edge e{(Vertex)i1, (Vertex)i2};
            elg.edges.push_back(e);
        }
        return elg;
    }

    template <class T = void>
    WeightedIndexedGraph<T> convert(const EdgeListGraph &graph)
    {
        WeightedIndexedGraph<T> gg;
        int ei = 1;
        for (auto e : graph.edges) {
            gg.addEdge(ei, e.u, e.v);
            ei++;
        }
        return gg;
    }

    /*template <class T>
    EdgeListGraph convert(const WeightedIndexedGraph<T> &graph, unsigned int vertexCounterStart = 1)
    {
        std::unordered_map<typename WeightedIndexedGraph<T>::Index, unsigned int> indexing;
        EdgeListGraph elg;
        elg.vertexCount = graph.nodeCount();
        elg.edgeCount = graph.edges.size();
        for (auto edge : graph.edges)
        {
            auto i1 = edge.second->leftNode->index;
            auto i2 = edge.second->rightNode->index;
            if (indexing.find(i1) == indexing.end())
            {
                indexing[i1] = vertexCounterStart+indexing.size();
            }
            if (indexing.find(i2) == indexing.end())
            {
                indexing[i2] = vertexCounterStart+indexing.size();
            }
            Edge e{indexing[i1], indexing[i2]};
            elg.edges.push_back(e);
        }
        return elg;
    }*/

    TreeDecomposition compute(const EdgeListGraph &graph, std::filesystem::path outputFile, bool enableSpecializedAlgorithms);
}


#endif //LINEPLANNING_TREEDECOMPOSITION_H
