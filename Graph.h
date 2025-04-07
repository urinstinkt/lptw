

#ifndef LINEPLANNING_GRAPH_H
#define LINEPLANNING_GRAPH_H

#include <unordered_map>
#include <vector>
#include <stdexcept>


template <class Node, class EW>
struct _Edge {
    int index;
    Node *leftNode, *rightNode;
    EW weight;

    _Edge(int index, Node *leftNode, Node *rightNode):index(index),leftNode(leftNode),rightNode(rightNode)
    {}
};
template <class Node>
struct _Edge<Node, void> {
    int index;
    Node *leftNode, *rightNode;

    _Edge(int index, Node *leftNode, Node *rightNode):index(index),leftNode(leftNode),rightNode(rightNode)
    {}
};

template <class EdgeWeight>
class WeightedIndexedGraph {
public:
    typedef int Index;

    struct Node;

    /*template <class EW>
    struct _Edge {
        Index index;
        Node *leftNode, *rightNode;
        EW weight;

        _Edge(Index index, Node *leftNode, Node *rightNode):index(index),leftNode(leftNode),rightNode(rightNode)
        {}
    };
    template <>
    struct _Edge<void> {
        Index index;
        Node *leftNode, *rightNode;

        _Edge(Index index, Node *leftNode, Node *rightNode):index(index),leftNode(leftNode),rightNode(rightNode)
        {}
    };*/

    typedef _Edge<Node, EdgeWeight> Edge;

    struct Node {
        Index index;
        std::unordered_map<Index, Edge*> incidentEdges;

        explicit Node(Index index):index(index)
        {}

        std::vector<std::pair<Edge*, Node*>> getNeighborhood() const
        {
            std::vector<std::pair<Edge*, Node*>> v;
            for (auto eit : incidentEdges)
            {
                Edge* e = eit.second;
                v.push_back({e, e->leftNode == this ? e->rightNode : e->leftNode});
            }
            return v;
        }

        std::vector<Node*> getNeighbors() const
        {
            std::vector<Node*> v;
            for (auto eit : incidentEdges)
            {
                Edge* e = eit.second;
                v.push_back(e->leftNode == this ? e->rightNode : e->leftNode);
            }
            return v;
        }
    };

    struct DuplicateEdgeException{
        Index edgeIndex;

        explicit DuplicateEdgeException(Index edgeIndex) : edgeIndex(edgeIndex) {}
    };

    std::unordered_map<Index, Node*> nodes;
    std::unordered_map<Index, Edge*> edges;

    WeightedIndexedGraph() = default;
    WeightedIndexedGraph(const WeightedIndexedGraph&) = delete;
    WeightedIndexedGraph(WeightedIndexedGraph&& source) noexcept
    {
        nodes = std::move(source.nodes);
        source.nodes = {};
        edges = std::move(source.edges);
        source.edges = {};
    }

    Node *getOrCreateNode(Index nodeIndex)
    {
        auto it = nodes.find(nodeIndex);
        if (it != nodes.end())
        {
            return it->second;
        }
        else
        {
            auto node = new Node(nodeIndex);
            nodes[nodeIndex] = node;
            return node;
        }
    }

    Edge *addEdge(Index edgeId, Index leftNode, Index rightNode)
    {
        if (edges.find(edgeId) != edges.end())
        {
            throw DuplicateEdgeException{edgeId};
        }
        Node* n1 = getOrCreateNode(leftNode);
        Node* n2 = getOrCreateNode(rightNode);
        Edge *edge = new Edge(edgeId, n1, n2);
        edges[edgeId] = edge;
        n1->incidentEdges[rightNode] = edge;
        n2->incidentEdges[leftNode] = edge;
        return edge;
    }

    Edge *getEdge(Index edgeId) const
    {
        auto it = edges.find(edgeId);
        if (it != edges.end())
            return it->second;
        return nullptr;
    }

    Edge *findEdge(Index v1, Index v2) const
    {
        auto it1 = nodes.find(v1);
        if (it1 != nodes.end())
        {
            auto ii = it1->second->incidentEdges.find(v2);
            if (ii != it1->second->incidentEdges.end())
                return ii->second;
        }

        /*auto it2 = nodes.find(v2);
        if (it2 != nodes.end())
        {
            auto ii = it2->second->incidentEdges.find(v1);
            if (ii != it2->second->incidentEdges.end())
                return ii->second;
        }*/

        return nullptr;
    }

    unsigned int nodeCount() const
    {
        return nodes.size();
    }

    ~WeightedIndexedGraph()
    {
        for (auto n : nodes)
        {
            delete n.second;
        }
        for (auto e : edges)
        {
            delete e.second;
        }
    }

    void indexNormalization(std::unordered_map<Index, Index> &renaming, std::unordered_map<Index, Index> &renamingInverse, Index startingIndex = 1) const {
        renaming.clear();
        renamingInverse.clear();
        for (const auto &p : nodes){
            renaming[p.first] = startingIndex;
            renamingInverse[startingIndex] = p.first;
            startingIndex++;
        }
    }
};

class CompactAdjacencyList{
public:
    std::vector<unsigned int> data;

    explicit CompactAdjacencyList(unsigned int linkCount);
};

template <class Weight>
class RootedNodeWeightedTree{
public:
    struct NodeData {
        Weight weight;
        unsigned int childCount;
        unsigned int adjacencyListIndex;

        bool isLeaf() const
        {
            return childCount == 0;
        }
    };

    typedef unsigned int NodeRef;

    std::vector<NodeData> nodes;
    CompactAdjacencyList al;

    struct Subtree{
        RootedNodeWeightedTree *tree;
        NodeRef subRoot;

        struct Iterator{
            Subtree subtree;
            unsigned int index;

            Subtree operator*(){
                return Subtree{subtree.tree, subtree.tree->al.data[subtree.tree->nodes[subtree.subRoot].adjacencyListIndex+index]};
            }
            void operator++(){
                index++;
            }
            bool operator!=(const Iterator &other){
                return index != other.index;
            }
        };

        Iterator begin()
        {
            return Iterator{*this, 0};
        }

        Iterator end()
        {
            return Iterator{*this, tree->nodes[subRoot].childCount};
        }

        bool isLeaf()
        {
            return tree->nodes[subRoot].childCount == 0;
        }

        Weight &weight()
        {
            return tree->nodes[subRoot].weight;
        }
    };

    explicit RootedNodeWeightedTree(unsigned int nodeCountReserved) : al(nodeCountReserved-1)
    {}

    Subtree getRoot()
    {
        return Subtree{this, 0};
    }

    template<class EW, class WeightConverter>
    static RootedNodeWeightedTree fromGraph(const WeightedIndexedGraph<EW> &graph, WeightConverter wc)
    {
        typedef WeightedIndexedGraph<EW> Graph;
        typedef typename Graph::Node GN;
        typedef typename Graph::Edge GE;
        //typedef std::pair<typename Graph::Node*, typename Graph::Edge*> W;
        RootedNodeWeightedTree tree(graph.nodeCount());
        unsigned int ai = 0;

        struct Work{
            GN* gn;
            GE* via;
            NodeRef parent;
        };
        std::vector<Work> worklist;
        GN* root = graph.nodes.begin()->second;
        worklist.push_back({root, nullptr, (unsigned int)-1});
        std::unordered_map<GN*, bool> added;
        added[root] = true;

        while (!worklist.empty())
        {
            Work work = worklist.back();
            worklist.pop_back();
            NodeRef treeNode = tree.nodes.size();
            if (work.via != nullptr)
            {
                tree.nodes.push_back(NodeData{wc(work.via),0,ai});
            }
            else
            {
                tree.nodes.push_back(NodeData{Weight(),0,ai});
            }
            if (work.parent != (unsigned int)-1)
            {
                tree.al.data[tree.nodes[work.parent].adjacencyListIndex+tree.nodes[work.parent].childCount] = treeNode;
                tree.nodes[work.parent].childCount++;
            }

            unsigned int viableEdges = 0;
            for (auto p : work.gn->getNeighborhood())
            {
                if (!added[p.second])
                {
                    viableEdges++;
                    worklist.push_back({p.second, p.first, treeNode});
                    added[p.second] = true;
                }
            }
            ai += viableEdges;
        }

        return tree;
    }

    template<class EW>
    static RootedNodeWeightedTree fromGraph(const WeightedIndexedGraph<EW> &graph)
    {
        auto wc = [](typename WeightedIndexedGraph<EW>::Edge *e){return Weight(e->weight);};
        return fromGraph(graph, wc);
    }
};

#endif //LINEPLANNING_GRAPH_H
