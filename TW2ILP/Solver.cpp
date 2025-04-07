
#include "Solver.h"
#include "PathPattern.h"
#include "../util.h"
#include <unordered_map>
#include <gurobi_c++.h>
#include <csignal>
#include <bitset>


using namespace LinePlanning;
using namespace std;

namespace Solver{

typedef GRBVar Var;
typedef int Vertex;
typedef vector<Vertex> Path;

bool isZero(const GRBLinExpr &expr){
    return expr.size() == 0;
}

unordered_map<char, unsigned int> varCounts;

int get(const Var& v){
    auto xd = v.get(GRB_DoubleAttr_X);
    int x = (int)xd;
    if (xd != (double)x) {
        cout << "warning: got non-integer value from Gurobi!" << endl;
        x = round(xd);
    }
    //cout << v.index() << ": " << x << endl;
    return x;
}

ostream& operator<<(ostream& os, std::set<int> s) {
    os << "{";
    for (auto it = s.begin(); it != s.end(); it++)
    {
        if (it != s.begin())
            os << ", ";
        os << *it;
    }
    os << "}";
    return os;
}

ostream& operator<<(ostream& os, Path p) {
    os << "(";
    for (auto it = p.begin(); it != p.end(); it++)
    {
        if (it != p.begin())
            os << ", ";
        os << *it;
    }
    os << ")";
    return os;
}

template <class PP>
class MappedPathCollection {

    unordered_map<PP, vector<pair<Path, unsigned int>>> paths;
    vector<pair<Path, unsigned int>> forgotten;
public:


    LineConcept toLC(const LinePlanning::Instance *instance) const{
        LineConcept lc;
        for (const auto& [_, pvec] : paths) {
            for (const auto& [path, freq] : pvec) {
                lc.appendLine(Line::fromVertexPath(freq, path, instance));
            }
        }
        for (const auto& [path, freq] : forgotten) {
            lc.appendLine(Line::fromVertexPath(freq, path, instance));
        }
        return lc;
    }

    /*constexpr static bool debug_log = false;
    void log() const {
        for (auto pair : paths) {
            cout << PathPatternVec{toVector(pair.first)} << endl;
            for (auto pathw : pair.second) {
                cout << "  " << pathw.first << " " << pathw.second << endl;
            }
        }
        cout << "()" << endl;
        for (auto pathw : forgotten) {
            cout << "  " << pathw.first << " " << pathw.second << endl;
        }
    }*/

    void add(const PP& pp, const Path& path, unsigned int count){
        paths[pp].push_back(make_pair(path, count));
        //if (debug_log)
        //    log();
    }

    void add(const MappedPathCollection &other){
        forgotten += other.forgotten;
        for (auto const& [pp, vec] : other.paths){
            paths[pp] += vec;
        }
        //if (debug_log)
        //    log();
    }

    template <class F>
    void forget(char toForget, F translation){
        unordered_map<PP, vector<pair<Path, unsigned int>>> newmap;
        for (auto [pp, vec] : paths) {
            auto opt = pp.forget(toForget);
            if (opt.has_value()){
                auto pp2 = opt.value().rename(translation);
                newmap[pp2] += vec;
            } else {
                forgotten += vec;
            }
        }
        paths = newmap;
        //if (debug_log)
        //    log();
    }

    template<class F>
    void rename(F translation){
        unordered_map<PP, vector<pair<Path, unsigned int>>> newmap;
        for (auto [pp, vec] : paths) {
            auto pp2 = pp.rename(translation);
            newmap[pp2] += vec;
        }
        paths = newmap;
        //if (debug_log)
        //    log();
    }

    void extendOrSubdivide(const PP& pp, const VertexRenaming<int,char> &vertexRenaming, char c1, char c2, char newV, unsigned int count) {
        if (c1 == PP::SQ) {
            std::swap(c1, c2);
        }
        while (count > 0) {
            auto &back = paths.at(pp).back();
            Path path2 = back.first;
            vector<char> dd = toVector(pp);
            auto it_dd = std::find(dd.begin(), dd.end(), c1);
            auto it_path = std::find(path2.begin(), path2.end(), vertexRenaming.inverse(c1));
            if (c2 == PP::SQ){
                if (it_dd == dd.begin()){
                    dd.insert(dd.begin(), newV);
                } else {
                    dd.push_back(newV);
                }
                if (it_path == path2.begin()){
                    path2.insert(path2.begin(), vertexRenaming.inverse(newV));
                } else {
                    path2.push_back(vertexRenaming.inverse(newV));
                }
            } else {
                if (it_dd+1 != dd.end() && *(it_dd+1) == c2){
                    dd.insert(it_dd+1, newV);
                } else {
                    dd.insert(it_dd, newV);
                }
                if (it_path+1 != path2.end() && *(it_path+1) == vertexRenaming.inverse(c2)){
                    path2.insert(it_path+1, vertexRenaming.inverse(newV));
                } else {
                    path2.insert(it_path, vertexRenaming.inverse(newV));
                }
            }
            auto pp2 = PP{dd};
            if (back.second <= count) {
                auto cc = back.second;
                count -= back.second;
                paths[pp].pop_back();
                paths[pp2].push_back({path2, cc});
            }
            else {
                auto cc = count;
                back.second -= count;
                paths[pp2].push_back({path2, cc});
                break;
            }
        }
        //if (debug_log)
        //    log();
    }

    void join(const PP& pp1, const PP& pp2, const PP& ppRes, unsigned int count, MappedPathCollection &other, const VertexRenaming<int,char> &vertexRenaming) {
        vector<char> mergePoints;
        for (char c : toVector(pp1)) {
            if (c != PP::SQ) {
                mergePoints.push_back(c);
            }
        }

        while (count > 0){
            auto &back1 = paths[pp1].back();
            auto &back2 = other.paths[pp2].back();
            auto actualCount = std::min({back1.second, back2.second, count});
            Path resPath;
            if (mergePoints.size() > 1) {
                Vertex v1 = vertexRenaming.inverse(mergePoints[0]);
                Vertex v2 = vertexRenaming.inverse(mergePoints[1]);
                if (std::find(back1.first.begin(), back1.first.end(), v1) > std::find(back1.first.begin(), back1.first.end(), v2)){
                    std::reverse(back1.first.begin(), back1.first.end());
                }
                if (std::find(back2.first.begin(), back2.first.end(), v1) > std::find(back2.first.begin(), back2.first.end(), v2)){
                    std::reverse(back2.first.begin(), back2.first.end());
                }
            } else // we merge Sx and Sx
            {
                Vertex v1 = vertexRenaming.inverse(mergePoints[0]);
                if ((std::find(back1.first.begin(), back1.first.end(), v1) == back1.first.begin()) ==
                        (std::find(back2.first.begin(), back2.first.end(), v1) == back2.first.begin())){
                    std::reverse(back1.first.begin(), back1.first.end());
                }
            };
            auto it1 = back1.first.begin();
            auto it2 = back2.first.begin();
            auto mpit = mergePoints.begin();
            while (mpit != mergePoints.end()){
                while (*it1 != vertexRenaming.inverse(*mpit)){
                    resPath.push_back(*it1);
                    it1++;
                }
                while (*it2 != vertexRenaming.inverse(*mpit)){
                    resPath.push_back(*it2);
                    it2++;
                }
                resPath.push_back(vertexRenaming.inverse(*mpit));
                it1++;
                it2++;
                mpit++;
            }
            while (it1 != back1.first.end()){
                resPath.push_back(*it1);
                it1++;
            }
            while (it2 != back2.first.end()){
                resPath.push_back(*it2);
                it2++;
            }

            add(ppRes, resPath, actualCount);
            back1.second -= actualCount;
            if (back1.second == 0){
                paths[pp1].pop_back();
            }
            back2.second -= actualCount;
            if (back2.second == 0){
                other.paths[pp2].pop_back();
            }
            count -= actualCount;
        }
        //if (debug_log)
        //    log();
    }

    void makeCycle(const PP& pp, unsigned int count) {
        while (count > 0) {
            auto &back = paths[pp].back();
            Path path2 = back.first;
            path2.push_back(path2[0]);
            auto actualCount = std::min({back.second, count});
            forgotten.push_back({path2, actualCount});
            back.second -= actualCount;
            if (back.second == 0){
                paths[pp].pop_back();
            }
            count -= actualCount;
        }
        //if (debug_log)
        //    log();
    }
};



template <class PP>
LinePlanning::LineConcept _solve(const Instance& instance, const TreeDecomposition::TreeDecomposition& td, const Options &options) {

    struct ReconstructionTree {

        struct Introduce;
        struct Forget;
        struct Join;
        struct Leaf;

        virtual void reconstruct(MappedPathCollection<PP> &mpc, VertexRenaming<int,char> &vertexRenaming) const = 0;
    };

    struct ReconstructionTree::Introduce : public ReconstructionTree {
        Vertex vertex;
        shared_ptr<ReconstructionTree> child;
        vector<tuple<PP,PP,Var>> e_vars;
        vector<tuple<PP,PP,Var>> s_vars;
        vector<tuple<PP,Var>> i_vars;

        void reconstruct(MappedPathCollection<PP> &mpc, VertexRenaming<int,char> &vertexRenaming) const override{
            child->reconstruct(mpc, vertexRenaming);
            vertexRenaming.add(vertex);

            for (const auto& [pp, var] : i_vars){
                auto vv = get(var);
                if (vv == 0)
                    continue;
                auto data = toVector(pp);
                mpc.add(pp, Path{{vertexRenaming.inverse(data[0]), vertexRenaming.inverse(data[1])}}, vv);
            }

            for (const auto& [pp1, pp2, var] : e_vars){
                auto vv = get(var);
                if (vv == 0)
                    continue;
                char c1;
                auto data = toVector(pp2);
                auto it = std::find(data.begin(), data.end(), vertexRenaming[vertex]);
                if (it == data.begin()){
                    c1 = *(it+1);
                } else {
                    c1 = *(it-1);
                }

                //debug
                //if (vertex == 22) {
                //    cout << "a";
                //}

                mpc.extendOrSubdivide(pp1, vertexRenaming, c1, PP::SQ, vertexRenaming[vertex], vv);
            }

            for (const auto& [pp1, pp2, var] : s_vars){
                auto vv = get(var);
                if (vv == 0)
                    continue;
                char c1,c2;
                auto data = toVector(pp2);
                auto it = std::find(data.begin(), data.end(), vertexRenaming[vertex]);
                c1 = *(it-1);
                c2 = *(it+1);
                mpc.extendOrSubdivide(pp1, vertexRenaming, c1, c2, vertexRenaming[vertex], vv);
            }
        }
    };

    struct ReconstructionTree::Forget : public ReconstructionTree {
        Vertex vertex;
        shared_ptr<ReconstructionTree> child;
        vector<tuple<PP,Var>> cycle_vars;

        void reconstruct(MappedPathCollection<PP> &mpc, VertexRenaming<int,char> &vertexRenaming) const override{
            child->reconstruct(mpc, vertexRenaming);

            for (const auto& [pp, var] : cycle_vars){
                auto vv = get(var);
                if (vv == 0)
                    continue;
                mpc.makeCycle(pp, vv);
            }

            auto translate = vertexRenaming.renamingCausedByErase(vertex);
            mpc.forget(vertexRenaming[vertex], translate);
            vertexRenaming.erase(vertex);
        }
    };

    struct ReconstructionTree::Join : public ReconstructionTree {
        shared_ptr<ReconstructionTree> child1, child2;
        vector<tuple<PP,PP,PP,Var>> j_vars;

        void reconstruct(MappedPathCollection<PP> &mpc, VertexRenaming<int,char> &vertexRenaming) const override{
            child1->reconstruct(mpc, vertexRenaming);
            MappedPathCollection<PP> mpc2;
            VertexRenaming<int,char> vertexRenaming2;
            child2->reconstruct(mpc2, vertexRenaming2);

            auto translate = [&](char c){
                return vertexRenaming[vertexRenaming2.inverse(c)];
            };
            mpc2.rename(translate);

            for (const auto& [pp1, pp2, ppRes, var] : j_vars){
                auto vv = get(var);
                if (vv == 0)
                    continue;
                mpc.join(pp1, pp2, ppRes, vv, mpc2, vertexRenaming);
            }

            mpc.add(mpc2);
        }
    };

    struct ReconstructionTree::Leaf : public ReconstructionTree {

        void reconstruct(MappedPathCollection<PP> &mpc, VertexRenaming<int,char> &vertexRenaming) const override{
        }
    };

    class NiceVisitor{

        set<int> vertices;
        const Instance* instance;
        GRBModel *model;
        const Options *options;

        unordered_map<PP, GRBLinExpr> c_expr;
        VertexRenaming<int,char> vertexRenaming;

        shared_ptr<ReconstructionTree> rTree;

    public:
        NiceVisitor(const Instance *instance, GRBModel *model, const Options *options) : instance(instance), model(model), options(options), rTree(make_shared<typename ReconstructionTree::Leaf>()) {}

        NiceVisitor(const NiceVisitor&) = delete;

        NiceVisitor(NiceVisitor&&) = default;

        void introduce(int v){
            //std::cout << "introduce node: " << vertices.size() << "+1" << endl;
            //std::cout << "introducing: " << v << endl;

            auto rTree = make_shared<typename ReconstructionTree::Introduce>();
            rTree->vertex = v;
            rTree->child = this->rTree;
            this->rTree = rTree;

            auto ppsNew = PP::allPatterns(vertices.size()+1);
            vertexRenaming.add(v);

            unordered_map<PP, GRBLinExpr> c_exprNew;
            unordered_map<PP, GRBLinExpr> c_rhs;
            for (const auto& pp : ppsNew){
                c_rhs[pp] = GRBLinExpr{};
            }

            for (const auto& [pp, var] : c_expr){
                c_rhs.at(pp) += var;
            }

            //introduce
            for (auto u : vertices){
                auto pp = PP{std::array<char,2>{vertexRenaming[u], vertexRenaming[v]}};
                auto var = model->addVar(0, GRB_INFINITY, instance->c_fix, GRB_INTEGER);
                varCounts['i']++;
                rTree->i_vars.push_back({pp, var});
                c_rhs.at(pp) += var;
            }

            //extend
            for (const auto& [pp, var] : c_expr){
                for (auto extension : pp.extensions(vertexRenaming[v])){
                    auto var = model->addVar(0, GRB_INFINITY, 0, GRB_INTEGER);
                    varCounts['e']++;
                    rTree->e_vars.push_back({pp, extension, var});
                    c_rhs.at(extension) += var;
                    c_rhs.at(pp) -= var;
                }
            }

            //subdivide
            for (const auto& [pp, _] : c_expr){
                for (auto sub : pp.subdivisions(vertexRenaming[v])){
                    auto var = model->addVar(0, GRB_INFINITY, 0, GRB_INTEGER);
                    varCounts['s']++;
                    rTree->s_vars.push_back({pp, sub, var});
                    c_rhs.at(sub) += var;
                    c_rhs.at(pp) -= var;
                }
            }

            for (const auto& [pp, r] : c_rhs){
                if (!isZero(r)) {
                    auto var = model->addVar(0, GRB_INFINITY, 0, GRB_INTEGER);
                    model->addConstr(r == var);
                    c_exprNew[pp] = var;
                } else {
                    c_exprNew[pp] = 0;
                }
            }

            vertices.insert(v);
            c_expr = c_exprNew;
        }
        void forget(int v){
            //std::cout << "forget node: " << vertices.size() << "-1" << endl;
            //std::cout << "forgetting: " << v << endl;

            auto rTree = make_shared<typename ReconstructionTree::Forget>();
            rTree->vertex = v;
            rTree->child = this->rTree;
            this->rTree = rTree;

            auto ppsNew = PP::allPatterns(vertices.size()-1);
            unordered_map<PP, GRBLinExpr> c_exprNew;
            unordered_map<PP, GRBLinExpr> c_rhs;
            for (const auto& pp : ppsNew){
                c_rhs[pp] = GRBLinExpr{};
            }
            vertices.erase(v);

            auto translate = vertexRenaming.renamingCausedByErase(v);

            if (options->allowCycles){
                for (const auto& [pp, cv] : c_expr){
                    if (endsWith(pp, PP::SQ))
                        continue;
                    if (!endsWith(pp, vertexRenaming[v]))
                        continue;
                    auto var = model->addVar(0, GRB_INFINITY, 0, GRB_INTEGER);
                    rTree->cycle_vars.push_back({pp, var});

                    model->addConstr(var <= cv);

                    auto ppn = pp.forget(vertexRenaming[v]);
                    auto ppnn = ppn.value().rename(translate);
                    c_rhs[ppnn] -= var;
                }
            }

            for (auto u : vertices){
                GRBLinExpr expr;
                for (const auto& [pp, cvar] : c_expr){
                    if (pp.containsEdge(vertexRenaming[u], vertexRenaming[v])){
                        expr += cvar;
                    }
                }
                if (options->allowCycles){
                    for (auto &[ppx, cycvar] : rTree->cycle_vars) {
                        if (PP(endings(ppx)).containsEdge(vertexRenaming[u], vertexRenaming[v])) {
                            expr += cycvar;
                        }
                    }
                }
                auto edge = instance->graph.findEdge(u, v);
                if (edge == nullptr)
                {
                    model->addConstr(expr == 0);
                }
                else
                {
                    auto vName = "f_"+to_string(u)+"_"+to_string(v);
                    auto var = model->addVar(edge->weight.f_min, edge->weight.f_max, edge->weight.cost, GRB_INTEGER, vName);
                    varCounts['f']++;
                    model->addConstr(expr == var);
                }
            }

            for (const auto& cv : c_expr){
                auto ppn = cv.first.forget(vertexRenaming[v]);
                if (ppn.has_value())
                {
                    auto ppnn = ppn.value().rename(translate);
                    c_rhs[ppnn] += cv.second;
                }
            }

            for (const auto& [pp, r] : c_rhs){
                if (!isZero(r)) {
                    auto var = model->addVar(0, GRB_INFINITY, 0, GRB_INTEGER);
                    model->addConstr(r == var);
                    c_exprNew[pp] = var;
                } else {
                    c_exprNew[pp] = 0;
                }
            }

            vertexRenaming.erase(v);
            c_expr = c_exprNew;
        }

        void merge(const NiceVisitor &other)
        {
            //std::cout << "join node: " << vertices.size() << endl;

            auto rTree = make_shared<typename ReconstructionTree::Join>();
            rTree->child1 = this->rTree;
            rTree->child2 = other.rTree;
            this->rTree = rTree;

            auto ppsNew = PP::allPatterns(vertices.size());
            unordered_map<PP, GRBLinExpr> c_exprNew;
            unordered_map<PP, GRBLinExpr> c_rhs;
            unordered_map<PP, GRBLinExpr> c_unjoined_1_rhs;
            unordered_map<PP, GRBLinExpr> c_unjoined_2_rhs;
            for (const auto& pp : ppsNew){
                c_rhs[pp] = GRBLinExpr{};
                c_unjoined_1_rhs[pp] = GRBLinExpr{};
                c_unjoined_2_rhs[pp] = GRBLinExpr{};
            }

            auto translate = [this, &other](char c){
                return vertexRenaming[other.vertexRenaming.inverse(c)];
            };

            for (const auto& [pp,v] : c_expr){
                c_unjoined_1_rhs[pp] += v;
            }
            for (const auto& [pp,v] : other.c_expr){
                auto ppt = pp.rename(translate);
                c_unjoined_2_rhs[ppt] += v;
            }

            for (const auto& [pp,v] : c_expr){
                for (const auto& [pp1, pp2] : pp.joins()){
                    if (!isZero(c_unjoined_1_rhs[pp1]) && !isZero(c_unjoined_2_rhs[pp2])){
                        auto var = model->addVar(0, GRB_INFINITY, -instance->c_fix, GRB_INTEGER);
                        varCounts['j']++;
                        rTree->j_vars.push_back({pp1, pp2, pp, var});
                        c_rhs[pp] += var;
                        c_unjoined_1_rhs[pp1] -= var;
                        c_unjoined_2_rhs[pp2] -= var;
                    }
                }
            }

            for (const auto& pp : ppsNew){
                c_rhs[pp] += c_unjoined_1_rhs[pp];
                c_rhs[pp] += c_unjoined_2_rhs[pp];
                model->addConstr(c_unjoined_1_rhs[pp] >= 0);
                model->addConstr(c_unjoined_2_rhs[pp] >= 0);
            }

            for (const auto& [pp, r] : c_rhs){
                if (!isZero(r)) {
                    auto var = model->addVar(0, GRB_INFINITY, 0, GRB_INTEGER);
                    model->addConstr(r == var);
                    c_exprNew[pp] = var;
                } else {
                    c_exprNew[pp] = 0;
                }
            }

            c_expr = c_exprNew;
        }

        auto reconstruct() const{
            MappedPathCollection<PP> mpc;
            VertexRenaming<int,char> vr;
            rTree->reconstruct(mpc, vr);
            return mpc;
        }
    };

    GRBEnv env;
    //env.set(GRB_IntParam_Presolve, 2); //aggressive presolve algorithm
#ifdef NDEBUG
    //env.set(GRB_IntParam_OutputFlag, 0);
#endif
    env.start();
    GRBModel model(env);
    static GRBModel* model_ptr = &model;

    Timer timerConsILP;
    NiceVisitor finishedVisitor = td.niceVisit([&](){
        return NiceVisitor(&instance, &model, &options);
        }, true);
    cout << "time to construct ILP: " << timerConsILP.get_string() << endl;

    cout << "variable counts:" << endl;
    for (auto [c, count] : varCounts) {
        cout << "  " << c << ": " << count << endl;
    }

    auto timeLimit = options.maxSolveTimeILP;

    constexpr bool cache_solution = false;
    model.update();
    int model_fingerprint = model.get(GRB_IntAttr_Fingerprint);
    if (cache_solution)
    {
        string sol_filename = "sol_"+std::to_string(model_fingerprint)+".sol";
        if (filesystem::exists(sol_filename)) {
            cout << "importing stored solution" << endl;
            model.read(sol_filename);
            model.update();
            timeLimit = 1;
        }
    }

    if (timeLimit != std::numeric_limits<double>::infinity()) {
        model.getEnv().set(GRB_DoubleParam_TimeLimit, timeLimit);
    }
    if (options.MIPGap >= 0) {
        model.getEnv().set(GRB_DoubleParam_MIPGap, options.MIPGap);
    }

    cout << "starting solver" << endl;
    cout << "press CTRL+C to stop solve" << endl;
    struct SignalThing {
        SignalThing() {
            auto signalHandler = [](int sig) {
                if (model_ptr != nullptr)
                    model_ptr->terminate();
            };
            signal(SIGINT, signalHandler);
        }
        ~SignalThing() {
            signal(SIGINT, SIG_DFL);
        }
    };
    SignalThing sh{};

    Timer timerSolver;

    bool outputPresolved = false;
    if (outputPresolved){
        auto premodel = model.presolve();
        premodel.write("presolve.lp");
        cout << "presolved system written to presolve.lp" << endl;
    }

    model.optimize();
    auto optimstatus = model.get(GRB_IntAttr_Status);
    if (optimstatus == GRB_OPTIMAL) {
        if (cache_solution)
        {
            string sol_filename = "sol_"+std::to_string(model_fingerprint)+".sol";
            cout << "saving solution to disk" << endl;
            model.write(sol_filename);
        }
    }
    else {
        cout << "optimization was stopped with status = " << optimstatus << endl;
    }
    if (optimstatus == GRB_INFEASIBLE) {
        throw InfeasibleInstanceError("instance is infeasible");
    }
    cout << "MIPGap: " << model.get(GRB_DoubleAttr_MIPGap) << endl;
#ifndef NDEBUG
    /*auto varCount = model.get(GRB_IntAttr_NumVars);
    for (int i = 0; i < varCount; i++){
        auto var = model.getVar(i);
        auto x = var.get(GRB_DoubleAttr_X);
        auto name = var.get(GRB_StringAttr_VarName);
        if (x != 0 && !name.starts_with("C")) {
            cout << name << ": " << x << endl;
        }
    }*/
#endif
    cout << "time to solve ILP: " << timerSolver.get_string() << endl;
    auto objVal = model.get(GRB_DoubleAttr_ObjVal);
    cout << "objective value: " << objVal << endl;

    Timer timerRecons;
    auto rec = finishedVisitor.reconstruct();
    auto lc = rec.toLC(&instance);
    cout << "time to create line concept from solution: " << timerRecons.get_string() << endl;
    return lc;
}


LinePlanning::LineConcept solve(const Instance& instance, const TreeDecomposition::TreeDecomposition& td, const Options &options) {
    auto requestedBagSize = td.getLargestBagSize();

    typedef PathPatternOptimized<unsigned int> PP1;
    typedef PathPatternOptimized<unsigned long long> PP2;
    //typedef PathPatternOptimized<std::bitset<128>> PP3;

    //for debugging
    //return _solve<PathPatternVec>(instance, td, options);
    //return _solve<PP1>(instance, td, options);


    if (requestedBagSize <= PP1::maxBagSize) {
        return _solve<PP1>(instance, td, options);
    } else if (requestedBagSize <= PP2::maxBagSize) {
        return _solve<PP2>(instance, td, options);
    } //else if (requestedBagSize <= PP3::maxBagSize) {
      //  return _solve<PP3>(instance, td, options);
    //}
    else {
        throw std::runtime_error("maximum supported treewidth exceeded");
    }
}

}
