
#include "PathPattern.h"

namespace LinePlanning {
    namespace Solver {

        PathPattern::edge_collection PathPattern::edges() const
        {
            return edge_collection{*this};
        }

        /*vector<PathPattern> PathPattern::allPathPatterns(vector<int> vertexSet) {
            if (vertexSet.empty())
            {
                return {};
            }
            auto v = vertexSet.back();
            vertexSet.pop_back();
            vector<PathPattern> vecReduced = allPathPatterns(vertexSet);
            vector<PathPattern> vec;
            for (const auto pr : vecReduced)
            {
                vec.push_back(pr);
                for (int i = 0; i < pr.size(); i++)
                {
                    if (pr.pat[i] == wildcard)
                    {
                        PathPattern pp = pr;
                        pp.pat[i] = v;
                        vec.push_back(pp);
                        //v is at i
                        pp.pat.insert(pp.pat.begin()+i, wildcard);
                        vec.push_back(pp);
                        //v is at i+1
                        pp.pat.insert(pp.pat.begin()+i+2, wildcard);
                        vec.push_back(pp);
                        //v is at i+1
                        pp.pat.erase(pp.pat.begin()+i);
                        vec.push_back(pp);
                    }
                }
            }
            vec.push_back({{v, wildcard}});
            vec.push_back({{wildcard, v}});
            vec.push_back({{wildcard, v, wildcard}});
            return vec;
        }

        set<PathPattern> PathPattern::allPathPatternsNormalized(vector<int> vertexSet) {
            auto v = allPathPatterns(vertexSet);
            return set<PathPattern>(v.begin(), v.end());
        }*/


        std::pair<bool, int> PathPattern::isSingleVertex() const {
            if (size() > 3)
            {
                return make_pair(false, 0);
            }
            int v = -1;
            for (int u : pat)
            {
                if (u != -1)
                {
                    if (v != -1)
                    {
                        return make_pair(false, 0);
                    }
                    else
                    {
                        v = u;
                    }
                }
            }
            return make_pair(true, v);
        }

        PathPattern::iterator PathPattern::findVertex(int v) const {
            return std::find(pat.begin(), pat.end(), v);
        }

        bool PathPattern::containsVertex(int v) const {
            return findVertex(v) != pat.end();
        }

        bool PathPattern::containsWildcard() const {
            return std::find(pat.begin(), pat.end(), wildcard) != pat.end();
        }

        bool PathPattern::incidentEdgesAreInsideBag(iterator it) const {
            if (it == pat.end())
                return true;
            return (it == pat.begin() || *(it-1) != -1) && ((it+1) == pat.end() || *(it+1) != -1);
        }

        vector<PathPattern> PathPattern::unReplace(int v) const {
            vector<PathPattern> vec;
            for (unsigned int i = 0; i < size(); i++)
            {
                if (pat[i] == wildcard)
                {
                    PathPattern pp1 = *this;
                    PathPattern pp2 = *this;
                    PathPattern pp3 = *this;
                    pp1.pat[i] = v;
                    pp2.pat.insert(pp2.pat.begin()+i, v);
                    pp3.pat.insert(pp3.pat.begin()+i+1, v);
                    vec.push_back(pp1);
                    vec.push_back(pp2);
                    vec.push_back(pp3);
                }
            }
            return vec;
        }

        PathPattern PathPattern::ofPath(vector<int> const& path, set<int> const& bagVertices) {
            vector<int> path_replaced;
            for (int v : path)
            {
                if (bagVertices.contains(v))
                {
                    path_replaced.push_back(v);
                }
                else
                {
                    if (path_replaced.empty() || path_replaced.back() != wildcard)
                        path_replaced.push_back(wildcard);
                }
            }
            return PathPattern{std::move(path_replaced)};
        }

        vector<unsigned int> PathPattern::matchesPathNoReversals(vector<int> const& path, set<int> const& bagVertices) const {
            vector<unsigned int> matchVec;
            unsigned int j = 0;
            vector<int> pp;
            bool ok = true;
            for (int index = 0; index < path.size(); index++)
            {
                auto v = path[index];
                if (bagVertices.contains(v))
                {
                    pp.push_back(v);
                    if (j < pat.size() && pp[j] == pat[j])
                    {
                        matchVec.push_back(j);
                        j++;
                    }
                    else
                    {
                        ok = false;
                        break;
                    }
                }
                else
                {
                    if (pp.empty() || pp.back() != wildcard)
                    {
                        pp.push_back(wildcard);
                        if (j < pat.size() && pp[j] == pat[j])
                        {
                            matchVec.push_back(j);
                            j++;
                        }
                        else
                        {
                            ok = false;
                            break;
                        }
                    }
                    else
                    {
                        matchVec.push_back(j-1);
                    }
                }
            }
            if (ok && j == size())
                return matchVec;
            return {};
        }

        vector<std::pair<PathPattern, PathPattern>> PathPattern::allNontrivialSplits() const {
            vector<std::pair<PathPattern, PathPattern>> splits;
            unsigned int wildcardCount = 0;
            for (unsigned int i = 0; i < pat.size(); i++)
            {
                if (pat[i] == wildcard)
                {
                    wildcardCount++;
                }
            }
            for (unsigned long state = 1; state < (1L << wildcardCount)-1; state++)
            {
                unsigned long ss = state;
                vector<int> p1, p2;
                for (unsigned int i = 0; i < pat.size(); i++)
                {
                    if (pat[i] == wildcard)
                    {
                        if (ss&1)
                        {
                            p1.push_back(wildcard);
                        }
                        else
                        {
                            p2.push_back(wildcard);
                        }
                        ss >>= 1;
                    }
                    else
                    {
                        p1.push_back(pat[i]);
                        p2.push_back(pat[i]);
                    }
                }
                splits.emplace_back(p1, p2);
            }
            return splits;
        }

        PathPattern::PathPattern(const vector<int> &pat) : pat(pat) {
        }

        PathPattern::PathPattern(vector<int> &&pat) : pat(std::move(pat)) {
        }

        std::ostream &operator<<(ostream &os, const PathPattern &pp) {
            for (auto v : pp.pat)
            {
                if (v == PathPattern::wildcard)
                    os << '?';
                else
                    os << v;
            }
            return os;
        }
    }
}