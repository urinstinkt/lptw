
#include "SpecializedTD.h"
#include <unordered_set>
#include <algorithm>

using namespace std;



namespace TreeDecomposition {

    struct hash_pair final {
        template<class TFirst, class TSecond>
        size_t operator()(const std::pair<TFirst, TSecond>& p) const noexcept {
            auto hash = std::hash<TFirst>{}(p.first);
            hash <<= sizeof(hash) * 4;
            hash ^= std::hash<TSecond>{}(p.second);
            return hash;
        }
    };

    typedef WeightedIndexedGraph<void>::Node Node;
    typedef WeightedIndexedGraph<void>::Index Index;

    auto commonNeighbors(Node* n1, Node* n2) {
        set<Node*> helper;
        vector<Node*> common;
        for (auto nb : n1->getNeighborhood()) {
            helper.insert(nb.second);
        }
        for (auto nb : n2->getNeighborhood()) {
            if (helper.contains(nb.second)) {
                common.push_back(nb.second);
            }
        }
        return common;
    }

    template <class C, class M>
    auto unmarkedNodes(const C& nodes, const M& marking) {
        vector<Node*> result;
        std::copy_if(nodes.begin(), nodes.end(), std::back_inserter(result), [&](Node* n) {return !marking.contains(n);});
        return result;
    }

    TreeDecomposition forGrid(const WeightedIndexedGraph<void> &graph) {
        try
        {
            //find degree 2 vertices
            vector<Node *> deg2Verts;
            for (auto p: graph.nodes)
            {
                if (p.second->incidentEdges.size() == 2)
                {
                    deg2Verts.push_back(p.second);
                }
            }
            if (deg2Verts.size() != 4)
            {
                throw WrongGraphClassError("not a grid graph");
            }

            typedef std::pair<int, int> GridPos;
            unordered_map<GridPos, Node *, hash_pair> grid;
            unordered_map<Node *, GridPos> gridR;
            auto nbh = deg2Verts[0]->getNeighborhood();
            grid[{0, 0}] = deg2Verts[0];
            grid[{1, 0}] = nbh[0].second;
            grid[{0, 1}] = nbh[1].second;
            gridR[deg2Verts[0]] = {0, 0};
            gridR[nbh[0].second] = {1, 0};
            gridR[nbh[1].second] = {0, 1};

            int level = 1;
            int maxX = 1;
            int maxY = 1;
            while (level <= maxY + maxX)
            {
                for (int i = level - maxX; i + 1 <= maxY; i++)
                {
                    auto n1 = grid.at({level - i, i});
                    auto n2 = grid.at({level - i - 1, i + 1});
                    auto cn = commonNeighbors(n1, n2);
                    for (auto nn: cn)
                    {
                        if (!gridR.contains(nn))
                        {
                            grid[{level - i, i + 1}] = nn;
                            gridR[nn] = {level - i, i + 1};
                            break;
                        }
                    }
                }
                if (level <= maxX)
                {
                    for (auto [_, nn]: grid.at({level, 0})->getNeighborhood())
                    {
                        if (!gridR.contains(nn))
                        {
                            grid[{level + 1, 0}] = nn;
                            gridR[nn] = {level + 1, 0};
                            maxX++;
                            break;
                        }
                    }
                }
                if (level <= maxY)
                {
                    for (auto [_, nn]: grid.at({0, level})->getNeighborhood())
                    {
                        if (!gridR.contains(nn))
                        {
                            grid[{0, level + 1}] = nn;
                            gridR[nn] = {0, level + 1};
                            maxY++;
                            break;
                        }
                    }
                }
                level++;
            }

            // check: all nodes found
            if (grid.size() != graph.nodes.size())
                throw WrongGraphClassError("not a grid graph");

            // check: is exactly a grid graph
            set<WeightedIndexedGraph<void>::Edge *> foundEdges;
            for (int y = 0; y <= maxY; y++)
            {
                for (int x = 0; x <= maxX; x++)
                {
                    if (x + 1 <= maxX)
                    {
                        auto e = graph.findEdge(grid.at({x + 1, y})->index, grid.at({x, y})->index);
                        if (e == nullptr)
                        {
                            throw WrongGraphClassError("not a grid graph");
                        }
                        else
                        {
                            foundEdges.insert(e);
                        }
                    }
                    if (y + 1 <= maxY)
                    {
                        auto e = graph.findEdge(grid.at({x, y + 1})->index, grid.at({x, y})->index);
                        if (e == nullptr)
                        {
                            throw WrongGraphClassError("not a grid graph");
                        }
                        else
                        {
                            foundEdges.insert(e);
                        }
                    }
                }
            }
            if (foundEdges.size() != graph.edges.size())
                throw WrongGraphClassError("not a grid graph");

            // tw = min(maxX+1,maxY+1)
            vector<set<Vertex>> bags;
            bool flip = maxX > maxY;
            if (flip)
            {
                decltype(grid) gridNew;
                for (auto p: grid)
                {
                    gridNew[{p.first.second, p.first.first}] = p.second;
                }
                grid = gridNew;
                std::swap(maxX, maxY);
            }
            set<Vertex> bag;
            for (int x = 0; x <= maxX; x++)
            {
                bag.insert(grid[{x, 0}]->index);
            }
            bags.push_back(bag);
            for (int y = 1; y <= maxY; y++)
            {
                for (int x = 0; x <= maxX; x++)
                {
                    bag.insert(grid[{x, y}]->index);
                    bags.push_back(bag);
                    bag.erase(grid[{x, y - 1}]->index);
                    bags.push_back(bag);
                }
            }
            return TreeDecomposition::fromPathDecomposition(bags);
        } catch (std::out_of_range &ex) {
            throw WrongGraphClassError("not a grid graph");
        }
    }


    TreeDecomposition forRings(const WeightedIndexedGraph<void> &graph) {
        typedef std::pair<int,int> GridPos; //(r, s)
        vector<Node*> nodesSorted;
        for (auto [_, n] : graph.nodes) {
            nodesSorted.push_back(n);
        }
        std::sort(nodesSorted.begin(), nodesSorted.end(), [](Node* n1, Node* n2) {
            return n1->incidentEdges.size() > n2->incidentEdges.size();
        });
        for (auto centerNode : nodesSorted) {
            try {
                if (centerNode->getNeighborhood().size() < 2)
                    continue;
                unordered_map<GridPos, Node*, hash_pair> grid;
                unordered_map<Node*, GridPos> gridR;

                grid[{-1,0}] = centerNode;
                gridR[centerNode] = {-1,0};
                int maxS = 0;
                int maxR = 0;
                int r = 0;
                while (1) {
                    auto nbh = unmarkedNodes((r > 0 ? grid.at({r-1,0}) : centerNode)->getNeighbors(), gridR);
                    if (nbh.empty()) {
                        break;
                    }
                    grid[{r,0}] = nbh[0];
                    gridR[nbh[0]] = {r,0};
                    int s = 1;
                    while (1) {
                        if (r > 0 && s>maxS)
                            break;
                        auto cn = unmarkedNodes(commonNeighbors(r > 0 ? grid.at({r-1,s}) : centerNode, grid.at({r,s-1})), gridR);
                        if (cn.empty())
                            break;
                        grid[{r,s}] = cn[0];
                        gridR[cn[0]] = {r,s};
                        maxS = max(s, maxS);
                        maxR = max(r, maxR);
                        s++;
                    }
                    r++;
                }

                // check: all nodes found
                if (grid.size() != graph.nodes.size())
                    throw WrongGraphClassError("not a ring graph");

                // check: is exactly a grid graph
                set<WeightedIndexedGraph<void>::Edge*> foundEdges;
                for (int r = -1; r <= maxR; r++) {
                    for (int s = 0; s <= maxS; s++) {
                        if (r+1 <= maxR) {
                            auto e = graph.findEdge(grid.at({r+1, s})->index, grid.at({r, r==-1?0:s})->index);
                            if (e == nullptr) {
                                throw WrongGraphClassError("not a ring graph");
                            } else {
                                foundEdges.insert(e);
                            }
                        }
                        if (r != -1) {
                            auto e = graph.findEdge(grid.at({r, (s+1)%(maxS+1)})->index, grid.at({r, s})->index);
                            if (e == nullptr) {
                                throw WrongGraphClassError("not a ring graph");
                            } else {
                                foundEdges.insert(e);
                            }
                        }
                    }
                }
                if (foundEdges.size() != graph.edges.size())
                    throw WrongGraphClassError("not a ring graph");

                // tw = min(2*maxR+3,maxS+1)
                vector<set<Vertex>> bags;
                if (maxS+1 < 2*maxR+3) {
                    set<Vertex> bag;
                    for (int s = 0; s <= maxS; s++) {
                        bag.insert(grid[{0, s}]->index);
                    }
                    bag.insert(grid[{-1, 0}]->index);
                    bags.push_back(bag);
                    bag.erase(grid[{-1, 0}]->index);
                    bags.push_back(bag);
                    for (int r = 1; r <= maxR; r++) {
                        for (int s = 0; s <= maxS; s++) {
                            bag.insert(grid[{r, s}]->index);
                            bags.push_back(bag);
                            bag.erase(grid[{r-1, s}]->index);
                            bags.push_back(bag);
                        }
                    }
                    return TreeDecomposition::fromPathDecomposition(bags);
                } else {
                    set<Vertex> bag;
                    for (int r = 0; r <= maxR; r++) {
                        bag.insert(grid[{r, 0}]->index);
                        bag.insert(grid[{r, 1}]->index);
                    }
                    bag.insert(grid[{-1, 0}]->index);
                    bags.push_back(bag);
                    for (int s = 2; s <= maxS; s++) {
                        for (int r = 0; r <= maxR; r++) {
                            bag.insert(grid[{r, s}]->index);
                            bags.push_back(bag);
                            bag.erase(grid[{r, s-1}]->index);
                            bags.push_back(bag);
                        }
                        bags.push_back(bag);
                    }
                    return TreeDecomposition::fromPathDecomposition(bags);
                }
            } catch (WrongGraphClassError &ex) {

            } catch (std::out_of_range &ex) {

            }
        }

        throw WrongGraphClassError("not a ring graph");
    }
}