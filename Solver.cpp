
#include "Solver.h"
#include <memory>
#include "Combinatorics.h"

/*#define THREADING
#ifdef THREADING
#include <omp.h>
#endif*/

namespace LinePlanning {
    namespace Solver {

        using TreeDecomposition::Bag;

        class LazyCostVector_Empty : public LazyCostVector{

        public:
            LazyCostVector_Empty() : LazyCostVector(nullptr) {}

            pair<ValueType,bool> compute_cost(const PathScheme &pathScheme) override {
                if (pathScheme.isZero())
                {
                    return {make_shared<AugmentedCost>(0), true};
                }
                else
                {
                    return {make_shared<AugmentedCost>(AugmentedCost::infinity()), true};
                }
            }
        };

        class LazyCostVector_Introduce : public LazyCostVector{

            int v;
            shared_ptr<LazyCostVector> cost_child;

        public:
            LazyCostVector_Introduce(const Instance *instance, int v, shared_ptr<LazyCostVector> costChild)
                    : LazyCostVector(instance), v(v), cost_child(costChild) {}

            pair<ValueType,bool> compute_cost(const PathScheme &pathScheme) override {
                cost_t costTransforms = 0;
                vector<BuildInstruction> transformations;
                PathScheme newPS;
                for (const auto &ps_it : pathScheme.data())
                {
                    auto pp = PathPattern{ps_it.first.someRepresentantion()};
                    auto vi = pp.findVertex(v);
                    if (vi != pp.end())
                    {
                        if (!pp.incidentEdgesAreInsideBag(vi))
                        {
                            return {make_shared<AugmentedCost>(AugmentedCost::infinity()), true}; //this is forbidden
                        }
                        else if (!pp.containsWildcard()) //all edges contained in bag
                        {
                            BuildInstruction transform;
                            transform.mode = BuildInstruction::CREATE;
                            transform.frequency = ps_it.second;
                            transform.result = pp;

                            transformations.push_back(transform);
                            costTransforms += transform.computeCost(instance);
                        }
                        else //have at least 3 vertices -> results from subdivide or append
                        {
                            BuildInstruction transform;
                            transform.mode = BuildInstruction::INSERT;
                            transform.frequency = ps_it.second;
                            transform.result = pp;
                            auto ppReduced = pp;
                            unsigned int vi_offset = vi-pp.begin();
                            ppReduced.erase(ppReduced.begin()+vi_offset);
                            transform.initial = ppReduced;
                            transform.insert_index = vi_offset;
                            transform.to_insert = v;

                            transformations.push_back(transform);
                            costTransforms += transform.computeCost(instance);
                            newPS.add(ppReduced, ps_it.second);
                        }
                    }
                    else
                    {
                        newPS.add(pp, ps_it.second);
                    }
                }
                auto r_child = cost_child->operator[](newPS);
                auto r = make_shared<AugmentedCost>(r_child);
                r->cost += costTransforms;
                r->instructions = std::move(transformations);
                r->bagChange = BagChange{BagChange::ADD, v};
                return {r, false};
            }
        };

        class LazyCostVector_forget : public LazyCostVector{

            set<int> B_t;
            int v;
            shared_ptr<LazyCostVector> cost_child;

        public:
            LazyCostVector_forget(const Instance *instance, const set<int> &B_t, int v, shared_ptr<LazyCostVector> costChild)
                    : LazyCostVector(instance), B_t(B_t), v(v), cost_child(costChild) {

            }

        private:

            bool hasForbiddenEdge(const PathPattern &pp)
            {
                auto vi = pp.findVertex(v);
                if (vi == pp.end())
                    return false;
                if (vi > pp.begin())
                {
                    auto u = *(vi-1);
                    if (u != PathPattern::wildcard && instance->graph.findEdge(u, v) == nullptr)
                        return true;
                }
                if (vi+1 != pp.end())
                {
                    auto u = *(vi+1);
                    if (u != PathPattern::wildcard && instance->graph.findEdge(u, v) == nullptr)
                        return true;
                }
                return false;
            }

        public:


            pair<ValueType,bool> compute_cost(const PathScheme &pathScheme) override {
                using Edge = InstanceGraph::Edge;
                using PPT = ReverseSymmetricVector<int>;

                Combinatorics::SequenceOfChoices<PPT> choices;
                for (const auto &it : pathScheme.data())
                {
                    vector<PPT> choices_tmp = {it.first};
                    auto unrep = PathPattern{it.first.someRepresentantion()}.unReplace(v);
                    for (const auto &pp : unrep)
                    {
                        if (!hasForbiddenEdge(pp))
                        {
                            auto ppt = PPT{pp.toVector()};
                            choices_tmp.push_back(ppt);
                        }
                    }
                    choices.choose(std::move(choices_tmp), it.second);
                }

                auto bestChild = make_shared<AugmentedCost>(AugmentedCost::infinity());
                for (auto choice : choices)
                {
                    PathScheme ps_child0;
                    unordered_map<Edge*, unsigned int> forgottenEdgeCount;
                    unordered_map<int, unsigned int> vertexFmaxCount;
                    for (auto &cv : choice)
                    {
                        for (auto &p : cv)
                        {
                            ps_child0.add(p.first, p.second);
                            PathPattern pp(p.first.someRepresentantion());
                            for (auto ed : pp.edges())
                            {
                                if (ed.first != PathPattern::wildcard && ed.second != PathPattern::wildcard && (ed.first == v || ed.second == v))
                                {
                                    Edge* e = instance->graph.findEdge(ed.first, ed.second);
                                    if (e != nullptr)
                                        forgottenEdgeCount[e] += p.second;
                                }
                                if (ed.first != PathPattern::wildcard)
                                    vertexFmaxCount[ed.first] += p.second;
                                if (ed.second != PathPattern::wildcard)
                                    vertexFmaxCount[ed.second] += p.second;
                            }
                        }
                    }
                    bool valid = true;

                    for (int u : B_t)
                    {
                        Edge* e = instance->graph.findEdge(u, v);
                        if (e != nullptr)
                        {
                            if (!(forgottenEdgeCount[e] >= e->weight.f_min && forgottenEdgeCount[e] <= e->weight.f_max))
                            {
                                valid = false;
                                break;
                            }
                        }
                    }
                    if (!valid)
                        continue;

                    for (auto p : vertexFmaxCount)
                    {
                        if (p.second > instance->getTotalFmax(p.first))
                        {
                            valid = false;
                            break;
                        }
                    }
                    if (!valid)
                        continue;

                    unsigned int v_allowance = instance->getTotalFmax(v) - vertexFmaxCount[v];
                    auto extra_pat1 = PPT{{v, PathPattern::wildcard}};
                    auto extra_pat2 = PPT{{PathPattern::wildcard, v, PathPattern::wildcard}};
                    for (unsigned int i1 = 0; i1 <= v_allowance; i1++)
                    {
                        for (unsigned int i2 = 0; i1+2*i2 <= v_allowance; i2++)
                        {
                            ps_child0.assign(extra_pat1, i1);
                            ps_child0.assign(extra_pat2, i2);
                            bestChild = std::min(bestChild, cost_child->operator[](ps_child0), [](auto p1, auto p2){return *p1 < *p2;});
                        }
                    }

                }
                auto result = make_shared<AugmentedCost>(bestChild);
                result->bagChange = BagChange{BagChange::REMOVE, v};
                return {result, false};
            }

            /*pair<ValueType,bool> compute_cost_old(const PathScheme &pathScheme)  {
                using Constraint = PathScheme::Constraint;
                using Edge = InstanceGraph::Edge;
                using PPT = ReverseSymmetricVector<int>;

                unordered_map<Edge*, Constraint> forgottenEdgeConstraints;
                for (int u : B_t)
                {
                    Edge* e = instance->graph.findEdge(u, v);
                    if (e != nullptr)
                    {
                        forgottenEdgeConstraints.insert({e, Constraint(e->weight.f_min, e->weight.f_max)});
                    }
                }

                map<PPT, Constraint> matchTargetSchemeConstraints;
                for (const auto &it : pathScheme.data())
                {
                    matchTargetSchemeConstraints.insert({it.first, Constraint(it.second, it.second)});
                }

                unordered_map<unsigned int, Constraint> totalFmaxBounds;
                totalFmaxBounds.insert({v, Constraint(0, instance->getTotalFmax(v))});
                for (int u : B_t)
                {
                    totalFmaxBounds.insert({u, Constraint(0, instance->getTotalFmax(u))});
                }

                set<PPT> patternsToConsider;
                for (const auto &it : pathScheme.data())
                {
                    patternsToConsider.insert(it.first);
                    matchTargetSchemeConstraints.at(it.first).insertThing(it.first);
                    auto tmp = PathPattern{it.first.someRepresentantion()}.unReplace(v);
                    for (const auto &pp : tmp)
                    {
                        if (!hasForbiddenEdge(pp))
                        {
                            auto ppt = PPT{pp.toVector()};
                            if (patternsToConsider.insert(ppt).second)
                                matchTargetSchemeConstraints.at(it.first).insertThing(ppt);
                        }
                    }
                }
                patternsToConsider.insert(PPT{{v, PathPattern::wildcard}});
                patternsToConsider.insert(PPT{{PathPattern::wildcard, v, PathPattern::wildcard}});

                for (const auto &pp : patternsToConsider)
                {
                    for (auto ed : PathPattern{pp.someRepresentantion()}.edges())
                    {
                        int u1 = ed.first;
                        int u2 = ed.second;
                        if (u1 != PathPattern::wildcard)
                        {
                            totalFmaxBounds.at(u1).insertThing(pp);
                        }
                        if (u2 != PathPattern::wildcard)
                        {
                            totalFmaxBounds.at(u2).insertThing(pp);
                        }
                        if (u1 != PathPattern::wildcard && u2 != PathPattern::wildcard)
                        {
                            if (u1 == v || u2 == v)
                            {
                                Edge *e = instance->graph.findEdge(u1, u2);
                                if (e != nullptr)
                                {
                                    forgottenEdgeConstraints.at(e).insertThing(pp);
                                }
                            }
                        }
                    }
                }

                vector<Constraint> allConstraints;
                for (auto [k,c] : forgottenEdgeConstraints)
                {
                    allConstraints.push_back(c);
                }
                for (auto [k,c] : matchTargetSchemeConstraints)
                {
                    allConstraints.push_back(c);
                }
                for (auto [k,c] : totalFmaxBounds)
                {
                    allConstraints.push_back(c);
                }
                auto allRecursivePathSchemes = PathScheme::allBoundedSchemes(patternsToConsider, allConstraints);
                auto bestChild = make_shared<AugmentedCost>(AugmentedCost::infinity());
                for (const auto& pp : allRecursivePathSchemes)
                {
                    bestChild = std::min(bestChild, cost_child->operator[](pp), [](auto p1, auto p2){return *p1 < *p2;});
                }
                auto result = make_shared<AugmentedCost>(bestChild);
                result->bagChange = BagChange{BagChange::REMOVE, v};
                return {result, false};
            }*/
        };

        class LazyCostVector_Join : public LazyCostVector{
            shared_ptr<LazyCostVector> child1, child2;
        public:

            LazyCostVector_Join(const Instance *instance, const shared_ptr<LazyCostVector> &child1,
                                const shared_ptr<LazyCostVector> &child2) : LazyCostVector(instance), child1(child1),
                                                                            child2(child2) {}

            pair<ValueType,bool> compute_cost(const PathScheme &pathScheme) override {

                struct Option{
                    enum {LEFT,RIGHT,MERGE} mode;
                    PathPattern left,right;
                };

                Combinatorics::SequenceOfChoices<Option> choices;
                for (auto it : pathScheme.data())
                {
                    PathPattern pp = it.first.someRepresentantion();
                    vector<Option> options;
                    {
                        Option opt;
                        opt.mode = Option::LEFT;
                        opt.left = pp;
                        options.push_back(opt);
                    }
                    {
                        Option opt;
                        opt.mode = Option::RIGHT;
                        opt.right = pp;
                        options.push_back(opt);
                    }
                    auto pp_splits = pp.allNontrivialSplits();
                    for (auto sp : pp_splits)
                    {
                        Option opt;
                        opt.mode = Option::MERGE;
                        opt.left = sp.first;
                        opt.right = sp.second;
                        options.push_back(opt);
                    }
                    choices.choose(std::move(options), it.second);
                }

                auto optimum = make_shared<AugmentedCost>(AugmentedCost::infinity());
                for (vector<vector<pair<Option,unsigned int>>> choice : choices)
                {
                    PathScheme ps_child1;
                    PathScheme ps_child2;
                    vector<BuildInstruction> transforms;
                    cost_t transform_costs = 0;
                    for (auto const& v : choice)
                    {
                        for (auto const& p : v)
                        {
                            switch (p.first.mode)
                            {
                                case Option::LEFT:
                                    ps_child1.add(p.first.left, p.second);
                                    break;
                                case Option::RIGHT:
                                    ps_child2.add(p.first.right, p.second);
                                    break;
                                case Option::MERGE:
                                    ps_child1.add(p.first.left, p.second);
                                    ps_child2.add(p.first.right, p.second);
                                    BuildInstruction transform;
                                    transform.mode = BuildInstruction::MERGE;
                                    transform.initial = p.first.left;
                                    transform.initial2 = p.first.right;
                                    transform.frequency = p.second;
                                    transform_costs += transform.computeCost(instance);
                                    transforms.push_back(std::move(transform));
                                    break;
                            }
                        }
                    }
                    auto cc = make_shared<AugmentedCost>(child1->operator[](ps_child1), child2->operator[](ps_child2));
                    cc->cost += transform_costs;
                    cc->instructions = std::move(transforms);
                    optimum = std::min(optimum, cc, [](auto p1, auto p2){return *p1 < *p2;});
                }
                return {optimum, false};
            }
        };

        struct NiceVisitor{

            set<int> vertices;
            shared_ptr<LazyCostVector> cost_current;
            const Instance* instance;

            NiceVisitor(const Instance *instance, const shared_ptr<LazyCostVector> &costCurrent) : cost_current(costCurrent),
                                                                                             instance(instance) {}

            void introduce(int v){
                cost_current = make_shared<LazyCostVector_Introduce>(instance, v, cost_current);
                std::cout << "introduce node: " << vertices.size() << "+1" << endl;
                vertices.insert(v);
            }
            void forget(int v){
                std::cout << "forget node: " << vertices.size() << "-1" << endl;
                vertices.erase(v);
                cost_current = make_shared<LazyCostVector_forget>(instance, vertices, v, cost_current);
            }

            void merge(const NiceVisitor &other)
            {
                std::cout << "join node: " << vertices.size() << endl;
                cost_current = make_shared<LazyCostVector_Join>(instance, cost_current, other.cost_current);
            }
        };

        shared_ptr<AugmentedCost> solve(const Instance &instance, const TreeDecomposition::TreeDecomposition &td) {
            NiceVisitor leafVisitor(&instance, make_shared<LazyCostVector_Empty>());
            NiceVisitor visitorFinished = td.niceVisit<NiceVisitor>(leafVisitor);
            auto res = visitorFinished.cost_current->operator[](PathScheme{});
            return res;
        }

        shared_ptr<AugmentedCost> solve(const Instance &instance) {
            TreeDecomposition::TreeDecomposition td = TreeDecomposition::compute(TreeDecomposition::convert(instance.graph));
            return solve(instance, td);
        }


        std::strong_ordering PathScheme::operator<=>(const PathScheme &other) const {
            auto it1 = _counts.cbegin();
            auto it2 = other._counts.cbegin();
            while (it1 != _counts.cend() || it2 != other._counts.cend())
            {
                strong_ordering x = strong_ordering::equal;
                if (it1 == _counts.end())
                {
                    x = strong_ordering::greater;
                }
                else if (it2 == other._counts.end())
                {
                    x = strong_ordering::less;
                }
                else
                {
                    x = it1->first <=> it2->first;
                }
                if (x == strong_ordering::less)
                {
                    strong_ordering y = it1->second <=> defaultValue;
                    if (y != strong_ordering::equal)
                        return y;
                    it1++;
                }
                else if (x == strong_ordering::greater)
                {
                    strong_ordering y = defaultValue <=> it2->second;
                    if (y != strong_ordering::equal)
                        return y;
                    it2++;
                }
                else
                {
                    strong_ordering y = it1->second <=> it2->second;
                    if (y != strong_ordering::equal)
                        return y;
                    it1++;
                    it2++;
                }
            }
            return strong_ordering::equal;
        }

        void PathScheme::operator+=(const PathScheme& other) {
            for (auto it : other._counts)
            {
                _counts[it.first] += it.second;
            }
        }

        bool PathScheme::isZero() const{
            for (auto it : _counts)
            {
                if (it.second != 0)
                    return false;
            }
            return true;
        }

        set<PathScheme> PathScheme::allBoundedSchemes(set<PPT> allowedPatterns, vector<Constraint> constraints) {
            if (allowedPatterns.empty())
            {
                for (auto& c : constraints)
                {
                    if (!c.isSatisfiedByAllZero())
                    {
                        return {};
                    }
                }
                return {PathScheme{}};
            }
            set<PathScheme> schemes;
            auto pp = *allowedPatterns.begin();
            allowedPatterns.erase(allowedPatterns.begin());
            bool exit = false;
            int val = 0;
            while (1)
            {
                vector<Constraint> constraints_new;
                for (const auto &c : constraints)
                {
                    Constraint cc = c;
                    cc.substitute(pp, val);
                    if (cc.isFalsified())
                    {
                        exit = true;
                        break;
                    }
                    if (!cc.guaranteedSatisfaction())
                    {
                        constraints_new.push_back(cc);
                    }
                }
                if (exit)
                    break;
                auto pss = allBoundedSchemes(allowedPatterns, constraints_new);
                for (const auto& ps : pss)
                {
                    PathScheme ps_copy = ps;
                    if (val != PathScheme::defaultValue)
                        ps_copy._counts[pp] = val;
                    schemes.insert(ps_copy);
                }
                val++;
            }
            return schemes;
        }

        void PathScheme::apply(BuildInstruction instruction, set<int> const &currentBag)
        {
            switch (instruction.mode)
            {
                case BuildInstruction::CREATE:
                    _counts[PPT{instruction.result.toVector()}] += instruction.frequency;
                    break;
                case BuildInstruction::INSERT:
                {
                    //cout << "BuildInstruction::INSERT" << endl;
                    //cout << instruction.initial << "->" << instruction.result << endl;
                    PathScheme toAdd;
                    auto to_reduce = instruction.frequency;
                    auto v = instruction.to_insert;
                    for (auto it = _counts.begin(); it != _counts.end() && to_reduce > 0; )
                    {
                        auto arepr = it->first.allRepresentantsions();
                        bool matched = false;
                        for (vector<int> consideredPath : {arepr.first, arepr.second})
                        {
                            auto match = instruction.initial.matchesPathNoReversals(consideredPath, currentBag);
                            if (!match.empty())
                            {
                                auto modifiedPath = consideredPath;
                                bool inserted = false;
                                if (instruction.insert_index == instruction.initial.size())
                                {
                                    modifiedPath.push_back(v);
                                    inserted = true;
                                }
                                else
                                {
                                    for (int i = 0; i < match.size(); i++)
                                    {
                                        auto mi = match[i];
                                        if (mi == instruction.insert_index)
                                        {
                                            modifiedPath.insert(modifiedPath.begin()+i, v);
                                            inserted = true;
                                            break;
                                        }
                                    }
                                }
                                if (!inserted)
                                {
                                    throw runtime_error("this should be impossible");
                                }
                                if (it->second > to_reduce)
                                {
                                    toAdd._counts[modifiedPath] += to_reduce;
                                    it->second -= to_reduce;
                                    to_reduce = 0;
                                    matched = true;
                                }
                                else
                                {
                                    toAdd._counts[modifiedPath] += it->second;
                                    to_reduce -= it->second;
                                    it = _counts.erase(it);
                                    matched = true;
                                }
                            }
                            if (matched)
                                break;
                        }
                        if (!matched)
                        {
                            it++;
                        }
                    }
                    if (to_reduce > 0)
                        throw runtime_error("failed to complete BuildInstruction");
                    operator+=(toAdd);
                    break;
                }
                case BuildInstruction::MERGE:
                    throw std::runtime_error("illegal case");
                    break;
            }
        }

        PathScheme PathScheme::mergeInstruction(PathScheme &ps1, PathScheme &ps2, BuildInstruction instruction,
                                                const set<int> &currentBag) {

            PathScheme result;
            vector<int> ip1 = instruction.initial.toVector();
            vector<int> ip2 = instruction.initial2.toVector();
            auto to_reduce = instruction.frequency;
            auto it1 = ps1._counts.begin();
            auto it2 = ps2._counts.begin();
            while (to_reduce > 0)
            {
                vector<int> consideredPath1, consideredPath2;
                vector<unsigned int> match1, match2;
                while (true){
                    auto arepr1 = it1->first.allRepresentantsions();

                    consideredPath1 = arepr1.first;
                    match1 = instruction.initial.matchesPathNoReversals(consideredPath1, currentBag);
                    if (match1.empty())
                    {
                        consideredPath1 = arepr1.second;
                        match1 = instruction.initial.matchesPathNoReversals(consideredPath1, currentBag);
                    }

                    if (match1.empty())
                        it1++;
                    else
                        break;
                }
                while (true){
                    auto arepr2 = it2->first.allRepresentantsions();

                    consideredPath2 = arepr2.first;
                    match2 = instruction.initial2.matchesPathNoReversals(consideredPath2, currentBag);
                    if (match2.empty())
                    {
                        consideredPath2 = arepr2.second;
                        match2 = instruction.initial2.matchesPathNoReversals(consideredPath2, currentBag);
                    }

                    if (match2.empty())
                        it2++;
                    else
                        break;
                }

                vector<int> resultPath;
                unsigned int i1 = 0, i2 = 0;
                while (true)
                {
                    while (i1 < match1.size() && ip1[match1[i1]] == PathPattern::wildcard)
                    {
                        resultPath.push_back(consideredPath1[i1]);
                        i1++;
                    }
                    while (i2 < match2.size() && ip2[match2[i2]] == PathPattern::wildcard)
                    {
                        resultPath.push_back(consideredPath2[i2]);
                        i2++;
                    }
                    if (i1 >= match1.size() && i2 >= match2.size())
                    {
                        break;
                    }
                    resultPath.push_back(consideredPath1[i1]);
                    i1++;
                    i2++;
                }
                unsigned int common = std::min(std::min(it1->second, it2->second), to_reduce);
                to_reduce -= common;
                it1->second -= common;
                it2->second -= common;
                result.add(PPT{resultPath}, common);

                if (it1->second == 0)
                {
                    it1 = ps1._counts.erase(it1);
                }
                if (it2->second == 0)
                {
                    it2 = ps2._counts.erase(it2);
                }
            }
            return result;
        }

        LineConcept PathScheme::toLineConcept(const Instance *instance) const {
            LineConcept lc;
            for (auto p : _counts)
            {
                lc += Line::fromVertexPath(p.second, p.first.someRepresentantion(), instance);
            }
            return lc;
        }

        ostream &operator<<(ostream &os, const PathScheme &ps) {
            for (auto [k,a] : ps._counts)
            {
                auto pp = PathPattern{k.someRepresentantion()};
                os << pp << ": " << a << endl;
            }
            return os;
        }

        LazyCostVector::LazyCostVector(const Instance *instance) : instance(instance) {}
    }
}