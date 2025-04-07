
#ifndef LINEPLANNING_PATHPATTERN_H
#define LINEPLANNING_PATHPATTERN_H

#include <vector>
#include <array>
#include <algorithm>
#include <iterator>
#include <unordered_map>
#include <unordered_set>
#include <optional>
#include <stdexcept>


template <class T>
void operator+=(std::vector<T> &v1, std::vector<T> const&v2){
    v1.insert(v1.end(), v2.begin(), v2.end());
}

using std::vector;


template <class T_orig, class T_target>
class VertexRenaming {
    std::unordered_map<T_orig, T_target> m1;
    std::unordered_map<T_target, T_orig> m2;

public:

    const T_target& operator[](T_orig v) const {
        return m1.at(v);
    }

    const T_orig& inverse(T_target vt) const {
        return m2.at(vt);
    }

    void add(T_orig v) {
        T_target vt;
        //if (!freeVals.empty()){
        //    auto it = freeVals.begin();
        //    vt = *it;
        //    freeVals.erase(it);
        //} else {
            vt = m1.size()+1;
        //}
        m1[v] = vt;
        m2[vt] = v;
    }

    auto renamingCausedByErase(T_orig v) const {
        auto it = m1.find(v);
        T_target vt = it->second;
        T_target s = m2.size();
        return [vt,s](T_target c){
            if (c == s){
                return vt;
            } else {
                return c;
            }
        };
    }

    void erase(T_orig v) {
        auto it = m1.find(v);
        T_target vt = it->second;
        m1.erase(it);
        if (vt != m2.size()){
            T_orig vo = m2[m2.size()];
            m1[vo] = vt;
            m2[vt] = vo;
        }
        m2.erase(m2.size());
    }
};

class PathPatternVec {

public:
    static constexpr char SQ = 0;
    vector<char> data;

    bool contains(char c) const {
        return std::find(data.begin(), data.end(), c) != data.end();
    }

    bool containsEdge(char c1, char c2) const {
        for (int i = 0; i+1 < data.size(); i++)
        {
            if (data[i] == c1 && data[i+1] == c2 || data[i] == c2 && data[i+1] == c1)
                return true;
        }
        return false;
    }

    std::optional<PathPatternVec> forget(char c) const {
        auto it = std::find(data.begin(), data.end(), c);
        if (it == data.end()){
            return *this;
        }
        else {
            vector<char> copy = data;
            auto i = it-data.begin();
            copy[i] = SQ;
            if (i+1 < copy.size() && copy[i+1] == SQ && i >= 1 && copy[i-1] == SQ){
                copy.erase(copy.begin()+i);
                copy.erase(copy.begin()+i);
            }
            else if (i+1 < copy.size() && copy[i+1] == SQ || i >= 1 && copy[i-1] == SQ){
                copy.erase(copy.begin()+i);
            }
            if (copy.size() >= 2)
                return PathPatternVec{copy};
            else
                return {};
        }
    }

    vector<PathPatternVec> extensions(char newV) const {
        vector<PathPatternVec> r;
        if (data[0] != SQ){
            vector<char> dd = data;
            dd.insert(dd.begin(), newV);
            r.push_back(PathPatternVec{dd});
        }
        if (data.back() != SQ){
            vector<char> dd = data;
            dd.push_back(newV);
            r.push_back(PathPatternVec{dd});
        }
        return r;
    }

    vector<PathPatternVec> subdivisions(char newV) const {
        vector<PathPatternVec> r;
        for (int i = 0; i+1 < data.size(); i++)
        {
            if (data[i] != SQ && data[i+1] != SQ){
                vector<char> dd = data;
                dd.insert(dd.begin()+(i+1), newV);
                r.push_back(PathPatternVec{dd});
            }
        }
        return r;
    }

    vector<std::pair<PathPatternVec,PathPatternVec>> joins() const {
        //special symmetric case: {SQ, x, SQ}
        if (data.size() == 3 && data[0] == SQ && data[2] == SQ){
            return {std::make_pair(PathPatternVec{{SQ, data[1]}}, PathPatternVec{{SQ, data[1]}})};
        }

        unsigned int squareCount = 0;
        for (int i = 0; i < data.size(); i++){
            if (data[i] == SQ){
                squareCount++;
            }
        }
        if (squareCount <= 1)
            return {};
        vector<std::pair<PathPatternVec,PathPatternVec>> r;
        for (unsigned int j = 0; j < (1<<squareCount); j++){
            vector<char> dd1, dd2;
            int k = 0;
            for (int i = 0; i < data.size(); i++){
                if (data[i] == SQ){
                    if ((j>>k)&1){
                        dd1.push_back(data[i]);
                    } else {
                        dd2.push_back(data[i]);
                    }
                    k++;
                }else {
                    dd1.push_back(data[i]);
                    dd2.push_back(data[i]);
                }
            }
            if (dd1.size() >= 2 && dd2.size() >= 2)
                r.push_back(std::make_pair(PathPatternVec{dd1}, PathPatternVec{dd2}));
        }
        return r;
    }

    template <class T>
    PathPatternVec rename(T renaming) const {
        vector<char> dd = data;
        for (auto& c : dd) {
            if (c == SQ)
                continue;
            c = renaming(c);
        }
        return PathPatternVec{dd};
    }

private:

    static bool isNormalized(vector<char> data) {
        auto r1 = data;
        std::reverse(r1.begin(), r1.end());
        return data <= r1;
    }

    static vector<char> normalize(vector<char> data) {
        auto r1 = data;
        std::reverse(r1.begin(), r1.end());
        return std::min(r1, data);
    }

    static vector<vector<char>> allPatternsInternal(int numVertices, int len) {
        if (len == 0)
            return {vector<char>{}};
        auto pps = allPatternsInternal(numVertices, len-1);
        vector<vector<char>> res;
        for (auto pp : pps){
            if (pp.empty() || pp.back() != SQ){
                auto pp2 = pp;
                pp2.push_back(SQ);
                res.push_back(pp2);
            }
            for (char c = 1; c <= numVertices; c++){
                if (std::find(pp.begin(), pp.end(), c) == pp.end()){
                    auto pp2 = pp;
                    pp2.push_back(c);
                    res.push_back(pp2);
                }
            }
        }
        return res;
    }

public:

    explicit PathPatternVec(const vector<char> &data) : data(normalize(data))  {
#ifndef NDEBUG
        if (data.size() < 2)
            throw 0;
#endif
    }

    template<size_t K>
    explicit PathPatternVec(const std::array<char,K> &arr) : PathPatternVec(vector<char>{arr.begin(), arr.end()}) {}

    static vector<PathPatternVec> allPatterns(int numVertices) {

        vector<PathPatternVec> res;
        for (int l = 2; l <= numVertices*2+1; l++){
            auto r = allPatternsInternal(numVertices, l);
            for (const auto &pp : r){
                if (isNormalized(pp)){
                    res.push_back(PathPatternVec{pp});
                }
            }
        }
        return res;
    }


    std::strong_ordering operator<=>(const PathPatternVec &other) const{
        return data <=> other.data;
    }

    bool operator==(const PathPatternVec &rhs) const {
        return data == rhs.data;
    }

    friend std::ostream& operator<<(std::ostream &os, const PathPatternVec &pp) {
        os << "(";
        for (auto it = pp.data.begin(); it != pp.data.end(); it++)
        {
            if (*it == SQ) {
                os << '?';
            } else {
                os << (int)*it;
            }
        }
        os << ")";
        return os;
    }
};

template<> struct std::hash<PathPatternVec>
{
    std::size_t operator()(PathPatternVec const& pp) const
    {
        return std::hash<std::string_view>()(std::string_view(pp.data.data(), pp.data.size()));
        //return std::_Hash_array_representation(pp.data.data(), pp.data.size());
    }
};


// compute ceil(log2(n))
constexpr size_t slog2(size_t n)
{
    return ( (n<=2) ? 1 : 1+slog2(n/2));
}

constexpr size_t spow(size_t a, size_t b)
{
    return ( (b==0) ? 1 : a*spow(a, b-1));
}


template <unsigned int B, class T>
constexpr unsigned int countDigits(T number, unsigned int minDigitCount, unsigned int maxDigitCount)
{
    if (minDigitCount == maxDigitCount)
        return minDigitCount;
    auto mid = (maxDigitCount+minDigitCount)/2;
    if (number < spow(B, mid)) {
        return countDigits<B,T>(number, minDigitCount, mid);
    } else {
        return countDigits<B,T>(number, mid+1, maxDigitCount);
    }
}

template <unsigned int B, class T>
constexpr T reverse_number(T number) {
    T r = 0;
    while (number != 0){
        r *= B;
        r += (number%B);
        number /= B;
    }
    return r;
}
template <unsigned int B, class T>
constexpr T reverse_number_fixed(T number, unsigned int numDigits) {
    T r = 0;
    for (int i = 0; i < numDigits; i++){
        r *= B;
        r += (number%B);
        number /= B;
    }
    return r;
}

/*class PathPatternOptimized {
    unsigned int data;
    friend std::hash<PathPatternOptimized>;

public:

    struct Context {
        unsigned int bagSize;
    };

    static vector<PathPatternOptimized> allPatterns() {

    }
};

template<> struct std::hash<PathPatternOptimized>
{
    std::size_t operator()(PathPatternOptimized const& pp) const
    {
        return std::hash<decltype(pp.data)>()(pp.data);
    }
};*/


/*
 * Path pattern = Permutation of a subset of all possible vertices, with some squares sprinkled in-between.
 * Encoding: Encode as a tuple of (sub_permutation, square_mask)
 */
template <class IntegerType>
class PathPatternOptimized {
    friend std::hash<PathPatternOptimized>;

    IntegerType data;

    //static_assert(_sqmask_bits+slog2(spow(maxBagSize+1, maxBagSize)) <= std::numeric_limits<Td>::digits, "maxBagSize too large!" );

    // s+1+slog2(spow(s+1, s)) <= bits

    static constexpr auto _maxBagSize(int bits) {
        int s = 1;
        int bitsRequired[] = {1, 3, 7, 10, 15, 19, 24, 29, 35, 40, 46, 52, 58, 64, 70, 76, 83, 89, 96, 103, 109, 116, 123, 130, 137, 144, 151, 158, 166, 173, 180};
        while (bitsRequired[s] <= bits) {
            s++;
        }
        return s-1;
    }


public:
    static auto constexpr maxBagSize = _maxBagSize(std::numeric_limits<decltype(data)>::digits);
private:
    static constexpr auto _sqmask_bits = maxBagSize+1;

    unsigned int properVertexCount() const {
        return countDigits<maxBagSize+1, decltype(data)>(_perm(), 1u, maxBagSize);
    }


    auto _perm() const {
        return data>>_sqmask_bits;
    }

    auto _sqmask() const {
        return data&IntegerType((1<<_sqmask_bits)-1);
    }

    auto _reversed() const {
        auto dc = properVertexCount();
        auto perm2 = reverse_number_fixed<maxBagSize+1>(_perm(), dc);
        auto sqmask2 = reverse_number_fixed<2>(_sqmask(), dc+1);
        return (perm2<<_sqmask_bits) | sqmask2;
    }

    PathPatternOptimized(IntegerType data) : data(data){

    }

public:
    static constexpr char SQ = 0;

    PathPatternOptimized(vector<char> vec){
        /*Td rdata[2];
        for (int k = 0; k < 2; k++){
            Td sqmask = 0;
            int si = 0;
            char verts[maxBagSize];
            for (int i = 0; i < vec.size(); i++){
                if (vec[i] != SQ){
                    verts[si] = vec[i];
                    si++;
                }
                else {
                    sqmask |= 1<<si;
                }
            }
            Td perm = 0;
            for (int i = 0; i < si; i++){
                perm *= maxBagSize+1;
                perm += verts[i];
            }
            rdata[k] = sqmask | (perm<<_sqmask_bits);
            if (k == 0) {
                std::reverse(vec.begin(), vec.end());
            }
        }
        this->data = std::min(rdata[0], rdata[1]);*/

        IntegerType sqmask = 0;
        int si = 0;
        char verts[maxBagSize];
        for (int i = 0; i < vec.size(); i++){
            if (vec[i] != SQ){
                verts[si] = vec[i];
                si++;
            }
            else {
                sqmask |= 1<<si;
            }
        }
        IntegerType perm = 0;
        for (int i = 0; i < si; i++){
            perm *= maxBagSize+1;
            perm += verts[i];
        }
        this->data = sqmask | (perm<<_sqmask_bits);
#ifndef NDEBUG
        // path pattern too short
        if (_perm() == 0 || (_perm() <= maxBagSize == 1 && _sqmask() == 0))
            throw 0;
#endif
    }

    PathPatternOptimized(std::initializer_list<char> l) : PathPatternOptimized(vector<char>{l}) {}
    template <std::size_t K>
    PathPatternOptimized(std::array<char,K> arr) : PathPatternOptimized(vector<char>{arr.begin(), arr.end()}) {}

    PathPatternOptimized(const PathPatternVec &ppv) : PathPatternOptimized(ppv.data) {}


    vector<char> toVec() const {
        vector<char> vec;
        vector<char> verts;
        auto perm = _perm();
        auto sqmask = _sqmask();
        while (perm != 0) {
            verts.push_back(perm%(maxBagSize+1));
            perm /= maxBagSize+1;
        }
        std::reverse(verts.begin(), verts.end());
        int si = verts.size();
        while (si >= 0) {
            if (sqmask & (1<<si)) {
                vec.push_back(SQ);
            }
            if (si >= 1)
                vec.push_back(verts[si-1]);
            si--;
        }
        return vec;
    }

    static std::unordered_set<PathPatternOptimized> allPatterns(int numVertices) {
        if (numVertices > maxBagSize)
            throw std::runtime_error("maxBagSize exceeded");
        std::unordered_set<PathPatternOptimized> res;
        /*for (int vc = 1; vc <= numVertices; vc++){

        }*/
        auto h = PathPatternVec::allPatterns(numVertices);
        for (auto pp : h) {
            res.insert(PathPatternOptimized{pp});
        }
        return res;
    }

    bool containsEdge(char c1, char c2) const {
        auto data = toVec();
        for (int i = 0; i+1 < data.size(); i++)
        {
            if (data[i] == c1 && data[i+1] == c2 || data[i] == c2 && data[i+1] == c1)
                return true;
        }
        return false;
    }

    std::optional<PathPatternOptimized> forget(char c) const {
        auto ppv = PathPatternVec(toVec());
        auto r = ppv.forget(c);
        if (r.has_value())
            return PathPatternOptimized(r.value());
        else
            return {};
    }

    vector<PathPatternOptimized> extensions(char newV) const {
        vector<PathPatternOptimized> r;
        vector<char> data = toVec();
        if (data[0] != SQ){
            vector<char> dd = data;
            dd.insert(dd.begin(), newV);
            r.push_back(PathPatternOptimized{dd});
        }
        if (data.back() != SQ){
            vector<char> dd = data;
            dd.push_back(newV);
            r.push_back(PathPatternOptimized{dd});
        }
        return r;
    }

    vector<PathPatternOptimized> subdivisions(char newV) const {
        vector<PathPatternOptimized> r;
        vector<char> data = toVec();
        for (int i = 0; i+1 < data.size(); i++)
        {
            if (data[i] != SQ && data[i+1] != SQ){
                vector<char> dd = data;
                dd.insert(dd.begin()+(i+1), newV);
                r.push_back(PathPatternOptimized{dd});
            }
        }
        return r;
    }

    vector<std::pair<PathPatternOptimized,PathPatternOptimized>> joins() const {
        unsigned int squareCount = std::popcount(_sqmask());
        if (squareCount <= 1)
            return {};

        //special symmetric case: {SQ, x, SQ}
        if (properVertexCount() == 1){
            auto v = (decltype(SQ))_perm();
            auto pp = PathPatternOptimized{{SQ, v}};
            return {std::make_pair(pp, pp)};
        }

        vector<char> data = toVec();
        vector<std::pair<PathPatternOptimized,PathPatternOptimized>> r;
        for (unsigned int j = 0; j < (1<<squareCount); j++){
            vector<char> dd1, dd2;
            int k = 0;
            for (int i = 0; i < data.size(); i++){
                if (data[i] == SQ){
                    if ((j>>k)&1){
                        dd1.push_back(data[i]);
                    } else {
                        dd2.push_back(data[i]);
                    }
                    k++;
                }else {
                    dd1.push_back(data[i]);
                    dd2.push_back(data[i]);
                }
            }
            if (dd1.size() >= 2 && dd2.size() >= 2)
                r.push_back(std::make_pair(PathPatternOptimized{dd1}, PathPatternOptimized{dd2}));
        }
        return r;
    }

    template <class T>
    PathPatternOptimized rename(T renaming) const {
        vector<char> dd = toVec();
        for (auto& c : dd) {
            if (c == SQ)
                continue;
            c = renaming(c);
        }
        return PathPatternOptimized{dd};
    }

    bool operator==(const PathPatternOptimized &rhs) const {
        return data == rhs.data || data == rhs._reversed();
    }
};

template<class T>
struct std::hash<PathPatternOptimized<T>>
{
    std::size_t operator()(PathPatternOptimized<T> const& pp) const
    {
        auto d = std::min(pp.data, pp._reversed());
        return std::hash<decltype(d)>()(d);
    }
};

vector<char> toVector(const PathPatternVec &pp) {
    return pp.data;
}

template <class T>
vector<char> toVector(const PathPatternOptimized<T> &pp) {
    return pp.toVec();
}

bool endsWith(const PathPatternVec &pp, char what) {
    return pp.data[0] == what || pp.data.back() == what;
}

template <class PP>
bool endsWith(const PP &pp, char what) {
    return endsWith(PathPatternVec{toVector(pp)}, what);
}

std::array<char,2> endings(const PathPatternVec &pp) {
    return {pp.data[0], pp.data.back()};
}

template <class PP>
std::array<char,2> endings(const PP &pp) {
    return endings(PathPatternVec{toVector(pp)});
}

/*template <class Vertex>
class MappedPath {
    PathPatternVec pp;
    vector<Vertex> path;
    vector<unsigned int> indexMap;

public:
    MappedPath(const PathPatternVec &pp, const VertexRenaming<Vertex, char> &renaming) : pp(pp){
        for (int i = 0; i < pp.data.size(); i++){
            if (pp.data[i] == PathPatternVec::SQ) {
                throw 0;
            }
            path.push_back(renaming.inverse(pp.data[i]));
            indexMap.push_back(i);
        }
    }

    MappedPath(const PathPatternVec &pp, const VertexRenaming<Vertex, char> &renaming, vector<Vertex> path) : pp(pp), path(path), indexMap(pp.data.size()){
        bool ok = false;
        for (int direction : {-1,1}) {
            int j = 0;
            int i = direction == 1 ? 0 : pp.data.size()-1;
            int end = direction == 1 ? pp.data.size() : -1;
            while (i != end && j < path.size()) {
                if (pp.data[i] == PathPatternVec::SQ){
                    indexMap[i] = j;
                    i += direction;
                } else {
                    if (renaming[path[j]] == pp.data[i]) {
                        indexMap[i] = j;
                        i += direction;
                    }
                }
                j++;
            }
            bool good = (i == end && (j == path.size() || pp.data[i-direction] == PathPatternVec::SQ));
            if (good)
            {
                ok = true;
                break;
            }
        }
    }

    operator PathPatternVec() const{
        return pp;
    }
};*/

#endif //LINEPLANNING_PATHPATTERN_H
