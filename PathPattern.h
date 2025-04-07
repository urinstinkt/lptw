

#ifndef LINEPLANNING_PATHPATTERN_H
#define LINEPLANNING_PATHPATTERN_H

#include <vector>
#include <set>
#include <iostream>

namespace LinePlanning {
    namespace Solver {

        using namespace std;

        template <class T>
        class ReverseSymmetricVector {
            vector<T> data;

            static pair<vector<T>,bool> normalized(vector<T> v){
                vector<T> r1 = v;
                std::reverse(r1.begin(), r1.end());
                if (r1 < v)
                {
                    return {r1, true};
                }
                else
                {
                    return {v, false};
                }
            }

        public:

            class iterator{
                friend ReverseSymmetricVector;
                typename vector<T>::const_iterator it;
            public:
                iterator(typename vector<T>::const_iterator it) : it(it)
                {
                }
                auto operator<=>(const iterator& other) const
                {
                    return it <=> other.it;
                }
                bool operator==(const iterator& other) const
                {
                    return it == other.it;
                }
                iterator operator+(unsigned int offset) const
                {
                    return {it+offset};
                }
                iterator operator-(unsigned int offset) const
                {
                    return {it-offset};
                }
                auto operator-(const iterator &other) const
                {
                    return it-other.it;
                }
                const T& operator*() const{
                    return *it;
                }
                void operator++()
                {
                    it++;
                }
                void operator--()
                {
                    it--;
                }
            };
            /*class const_iterator{
                friend ReverseSymmetricVector;
                typename vector<T>::const_iterator it;
            public:
                const_iterator(typename vector<T>::const_iterator it) : it(it)
                {
                }
                auto operator<=>(const iterator& other) const
                {
                    return it <=> other.it;
                }
                auto operator==(const iterator& other) const
                {
                    return it == other.it;
                }
                iterator operator+(unsigned int offset)
                {
                    return {it+offset};
                }
                iterator operator-(unsigned int offset)
                {
                    return {it-offset};
                }
                const T& operator*() const{
                    return *it;
                }
                void operator++()
                {
                    it++;
                }
                void operator--()
                {
                    it--;
                }
            };*/

            auto size() const{
                return data.size();
            }

        private:
            iterator reversed(iterator pos)
            {
                auto offset = pos.it-data.begin();
                return {data.begin()+offset};
            }
        public:

            ReverseSymmetricVector(const vector<T> &data = {}) : data(normalized(data).first)
            {}

            /*iterator set(iterator pos, const T &value)
            {
                auto offset = pos.it-data.begin();
                data[offset] = value;
                auto p = normalized(data);
                if (p.second)
                {
                    data = p.first;
                    return reversed(pos);
                }
                else
                {
                    return pos;
                }
            }

            iterator insert(iterator pos, const T &value)
            {
                data.insert(pos.it, value);
                auto p = normalized(data);
                if (p.second)
                {
                    data = p.first;
                    return reversed(pos);
                }
                else
                {
                    return pos;
                }
            }

            iterator find(const T &value) const
            {
                auto it = std::find(data.begin(), data.end(), value);
                return iterator{it};
            }*/

            iterator begin() const
            {
                auto it = data.begin();
                return iterator{it};
            }

            iterator end() const
            {
                auto it = data.end();
                return iterator{it};
            }

            const vector<T>& someRepresentantion() const{
                return data;
            }

            const pair<vector<T>,vector<T>> allRepresentantsions() const{
                vector<T> r = data;
                std::reverse(r.begin(), r.end());
                return {data, r};
            }

            std::strong_ordering operator<=>(ReverseSymmetricVector const& other) const
            {
                return data <=> other.data;
            }
        };

        class PathPattern {
        protected:
            vector<int> pat;
        public:
            typedef vector<int>::const_iterator iterator;
            static constexpr int wildcard = -1;

            friend class BuildInstruction;

            PathPattern(const vector<int> &pat = {});
            PathPattern(vector<int> &&pat);

            auto size() const{
                return pat.size();
            }

            struct edge_iterator{
                iterator it;

                pair<int,int> operator*() const{
                    return make_pair(*it, *(it+1));
                }

                void operator++(){
                    it++;
                }

                bool operator==(const edge_iterator& other) const{
                    return it == other.it;
                }
            };

            struct edge_collection{
                const PathPattern &pp;
                edge_iterator begin() const{
                    return edge_iterator{pp.pat.begin()};
                }
                edge_iterator end() const{
                    return edge_iterator{pp.pat.end()-1};
                }
            };

            //includes wildcard-vertex pairs
            edge_collection edges() const;

            //static vector<PathPattern> allPathPatterns(vector<int> vertexSet);
            //static set<PathPattern> allPathPatternsNormalized(vector<int> vertexSet);

            static PathPattern ofPath(vector<int> const& path, set<int> const& bagVertices);
            vector<unsigned int> matchesPathNoReversals(vector<int> const& path, set<int> const& bagVertices) const;

            std::strong_ordering operator<=>(const PathPattern &other) const = delete;

            std::pair<bool,int> isSingleVertex() const;

            iterator findVertex(int v) const;
            bool containsVertex(int v) const;
            bool containsWildcard() const;

            bool incidentEdgesAreInsideBag(iterator it) const;


            vector<PathPattern> unReplace(int v) const;

            vector<std::pair<PathPattern, PathPattern>> allNontrivialSplits() const;

            const vector<int> &toVector() const
            {
                return pat;
            }

            iterator begin() const{
                return pat.begin();
            }

            iterator end() const {
                return pat.end();
            }

            void erase(iterator it){
                pat.erase(it);
            }


            friend std::ostream& operator<<(std::ostream& os, const PathPattern &pp);
        };

    }
}

#endif //LINEPLANNING_PATHPATTERN_H
