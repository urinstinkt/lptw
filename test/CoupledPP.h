
#ifndef LINEPLANNING_COUPLEDPP_H
#define LINEPLANNING_COUPLEDPP_H

#include <set>
#include "../TW2ILP/PathPattern.h"

typedef PathPatternOptimized<unsigned int> PPO;

using namespace std;

/*
 * Class for debugging/testing; asserts that two PP data structures behave the same way.
 */
class CoupledPP {

public:
    PPO my_ppo;
    PathPatternVec my_ppv;

    static constexpr char SQ = 0;

    CoupledPP(PPO ppo, PathPatternVec ppv) : my_ppo(ppo), my_ppv(ppv) {
        check();
    }

    CoupledPP(vector<char> raw) : my_ppo(raw), my_ppv(raw)  {
        check();
    }

    CoupledPP(array<char, 2> raw) : my_ppo(raw), my_ppv(raw)  {
        check();
    }

    void check() const {
        auto v1 = my_ppv.data;
        auto v2 = v1;
        std::reverse(v2.begin(), v2.end());
        auto v3 = my_ppo.toVec();
        if (v1 != v3 && v2 != v3)
            throw runtime_error("");
    }

    static CoupledPP couple(const PPO &ppo, const PathPatternVec &ppv) {
        CoupledPP pp{ppo, ppv};
        return pp;
    }

    template <class T>
    static T assert_eq_and_return(T t1, T t2) {
        if (t1 != t2)
            throw runtime_error("");
        return t1;
    }

    template <class Container1, class Container2>
    static vector<CoupledPP> coupleAll(const Container1 &ppos, const Container2 &ppvs) {
        set<PathPatternVec> s1{ppvs.begin(), ppvs.end()};
        set<PathPatternVec> s2;
        for (auto pp : ppos) {
            s2.insert(PathPatternVec{toVector(pp)});
        }
        vector<CoupledPP> r;
        for (auto pp : assert_eq_and_return(s1, s2)) {
            r.emplace_back(toVector(pp));
        }
        return r;
    }

    bool containsEdge(char c1, char c2) const {
        return assert_eq_and_return(my_ppo.containsEdge(c1, c2), my_ppv.containsEdge(c1,c2));
    }

    auto operator==(const CoupledPP &o) const {
        return assert_eq_and_return(my_ppo == o.my_ppo, my_ppv == o.my_ppv);
    }

    std::optional<CoupledPP> forget(char c) const {
        auto o1 = my_ppo.forget(c);
        auto o2 = my_ppv.forget(c);
        if (assert_eq_and_return(o1.has_value(),o2.has_value())) {
            return couple(o1.value(), o2.value());
        } else {
            return {};
        }
    }

    vector<CoupledPP> extensions(char newV) const {
        return coupleAll(my_ppo.extensions(newV), my_ppv.extensions(newV));
    }

    vector<CoupledPP> subdivisions(char newV) const {
        return coupleAll(my_ppo.subdivisions(newV), my_ppv.subdivisions(newV));
    }

    vector<std::pair<CoupledPP,CoupledPP>> joins() const {
        throw runtime_error("");

        vector<std::pair<CoupledPP,CoupledPP>> r;
        //unchecked
        for (auto p : my_ppo.joins()) {
            r.push_back(make_pair(CoupledPP{toVector(p.first)}, CoupledPP{toVector(p.second)}));
        }
        return r;
    }

    template <class T>
    CoupledPP rename(T renaming) const {
        return couple(my_ppo.rename(renaming), my_ppv.rename(renaming));
    }

    static vector<CoupledPP> allPatterns(int numVertices) {
        return coupleAll(decltype(my_ppo)::allPatterns(numVertices), decltype(my_ppv)::allPatterns(numVertices));
    }
};

vector<char> toVector(const CoupledPP &pp) {
    return toVector(pp.my_ppv);
}


template<> struct std::hash<CoupledPP>
{
    std::size_t operator()(CoupledPP const& pp) const
    {
        return std::hash<PPO>()(pp.my_ppo);
    }
};

#endif //LINEPLANNING_COUPLEDPP_H
