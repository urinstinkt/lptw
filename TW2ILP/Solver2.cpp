
#include "Solver2.h"
#include "PathPattern.h"
#include <unordered_map>
#include <gurobi_c++.h>

using namespace LinePlanning;
using namespace std;

namespace Solver{
    typedef GRBVar Var;
    typedef PathPatternVec PP;
    typedef int Vertex;
    typedef vector<Vertex> Path;

    LinePlanning::LineConcept solve2(const LinePlanning::Instance& instance, const TreeDecomposition::TreeDecomposition& td){

        class NiceVisitor{

            set<int> vertices;
            const Instance* instance;
            GRBModel *model;

            unordered_map<PathPatternVec, Var> c_vars;
            VertexRenaming<int,char> vertexRenaming;

        public:
            NiceVisitor(const Instance *instance, GRBModel *model) : instance(instance), model(model), rTree(make_shared<ReconstructionTree::Leaf>()) {}

            void introduce(int v){
                //std::cout << "introduce node: " << vertices.size() << "+1" << endl;

                auto rTree = make_shared<ReconstructionTree::Introduce>();
                rTree->vertex = v;
                rTree->child = this->rTree;
                this->rTree = rTree;

                auto ppsNew = PathPatternVec::allPatterns(vertices.size()+1);
                vertexRenaming.add(v);

                unordered_map<PathPatternVec, Var> c_varsNew;
                unordered_map<PathPatternVec, GRBLinExpr> c_rhs;
                for (const auto& pp : ppsNew){
                    c_varsNew[pp] = model->addVar(0, GRB_INFINITY, 0, GRB_INTEGER);
                    c_rhs[pp] = GRBLinExpr{};
                }

                for (const auto& [pp, var] : c_vars){
                    c_rhs.at(pp) += var;
                }

                //introduce
                for (auto u : vertices){
                    auto pp = PathPatternVec{{vertexRenaming[u], vertexRenaming[v]}};
                    auto var = model->addVar(0, GRB_INFINITY, instance->c_fix, GRB_INTEGER);
                    rTree->i_vars.push_back({pp, var});
                    c_rhs.at(pp) += var;
                }

                //extend
                for (const auto& [pp, var] : c_vars){
                    for (auto extension : pp.extensions(vertexRenaming[v])){
                        auto var = model->addVar(0, GRB_INFINITY, 0, GRB_INTEGER);
                        rTree->e_vars.push_back({pp, extension, var});
                        c_rhs.at(extension) += var;
                        c_rhs.at(pp) -= var;
                    }
                }

                //subdivide
                for (const auto& [pp, _] : c_vars){
                    for (auto sub : pp.subdivisions(vertexRenaming[v])){
                        auto var = model->addVar(0, GRB_INFINITY, 0, GRB_INTEGER);
                        rTree->s_vars.push_back({pp, sub, var});
                        c_rhs.at(sub) += var;
                        c_rhs.at(pp) -= var;
                    }
                }

                for (const auto& [pp, r] : c_rhs){
                    model->addConstr(r == c_varsNew.at(pp));
                }

                vertices.insert(v);
                c_vars = c_varsNew;
            }
            void forget(int v){
                //std::cout << "forget node: " << vertices.size() << "-1" << endl;

                auto rTree = make_shared<ReconstructionTree::Forget>();
                rTree->vertex = v;
                rTree->child = this->rTree;
                this->rTree = rTree;

                auto ppsNew = PathPatternVec::allPatterns(vertices.size()-1);
                unordered_map<PathPatternVec, Var> c_varsNew;
                unordered_map<PathPatternVec, GRBLinExpr> c_rhs;
                for (const auto& pp : ppsNew){
                    c_varsNew[pp] = model->addVar(0, GRB_INFINITY, 0, GRB_INTEGER);
                    c_rhs[pp] = GRBLinExpr{};
                }
                vertices.erase(v);

                auto translate = vertexRenaming.renamingCausedByErase(v);

                for (auto u : vertices){
                    GRBVar var;
                    auto edge = instance->graph.findEdge(u, v);
                    if (edge == nullptr)
                    {
                        var = model->addVar(0, 0, 0, GRB_INTEGER);
                    }
                    else
                    {
                        var = model->addVar(edge->weight.f_min, edge->weight.f_max, edge->weight.cost, GRB_INTEGER);
                    }
                    GRBLinExpr expr;
                    for (const auto& [pp, cvar] : c_vars){
                        if (pp.containsEdge(vertexRenaming[u], vertexRenaming[v])){
                            expr += cvar;
                        }
                    }
                    model->addConstr(expr == var);
                }

                for (const auto& cv : c_vars){
                    auto ppn = cv.first.forget(vertexRenaming[v]);
                    if (ppn.has_value())
                    {
                        auto ppnn = ppn.value().rename(translate);
                        c_rhs[ppnn] += cv.second;
                    }
                }
                for (const auto& [pp, rhs] : c_rhs){
                    model->addConstr(rhs== c_varsNew[pp]);
                }

                vertexRenaming.erase(v);
                c_vars = c_varsNew;
            }

            void merge(const NiceVisitor &other)
            {
                //std::cout << "join node: " << vertices.size() << endl;

                auto rTree = make_shared<ReconstructionTree::Join>();
                rTree->child1 = this->rTree;
                rTree->child2 = other.rTree;
                this->rTree = rTree;

                auto ppsNew = PathPatternVec::allPatterns(vertices.size());
                unordered_map<PathPatternVec, Var> c_varsNew;
                unordered_map<PathPatternVec, GRBLinExpr> c_rhs;
                unordered_map<PathPatternVec, Var> c_unjoined_1;
                unordered_map<PathPatternVec, GRBLinExpr> c_unjoined_1_rhs;
                unordered_map<PathPatternVec, Var> c_unjoined_2;
                unordered_map<PathPatternVec, GRBLinExpr> c_unjoined_2_rhs;
                for (const auto& pp : ppsNew){
                    c_varsNew[pp] = model->addVar(0, GRB_INFINITY, 0, GRB_INTEGER);
                    c_unjoined_1[pp] = model->addVar(0, GRB_INFINITY, 0, GRB_INTEGER);
                    c_unjoined_2[pp] = model->addVar(0, GRB_INFINITY, 0, GRB_INTEGER);
                    c_rhs[pp] = GRBLinExpr{};
                    c_unjoined_1_rhs[pp] = GRBLinExpr{};
                    c_unjoined_2_rhs[pp] = GRBLinExpr{};
                }

                auto translate = [this, &other](char c){
                    return vertexRenaming[other.vertexRenaming.inverse(c)];
                };

                for (const auto& [pp,v] : c_vars){
                    c_unjoined_1_rhs[pp] += v;
                }
                for (const auto& [pp,v] : other.c_vars){
                    auto ppt = pp.rename(translate);
                    c_unjoined_2_rhs[ppt] += v;
                }

                for (const auto& [pp,v] : c_vars){
                    for (const auto& [pp1, pp2] : pp.joins()){
                        auto var = model->addVar(0, GRB_INFINITY, -instance->c_fix, GRB_INTEGER);
                        rTree->j_vars.push_back({pp1, pp2, pp, var});
                        c_rhs[pp] += var;
                        c_unjoined_1_rhs[pp1] -= var;
                        c_unjoined_2_rhs[pp2] -= var;
                    }
                }

                for (const auto& pp : ppsNew){
                    c_rhs[pp] += c_unjoined_1[pp]+c_unjoined_2[pp];
                    model->addConstr(c_rhs[pp] == c_varsNew[pp]);
                    model->addConstr(c_unjoined_1_rhs[pp] == c_unjoined_1[pp]);
                    model->addConstr(c_unjoined_2_rhs[pp] == c_unjoined_2[pp]);
                }

                c_vars = c_varsNew;
            }

            MappedPathCollection reconstruct() const{
                MappedPathCollection mpc;
                VertexRenaming<int,char> vr;
                rTree->reconstruct(mpc, vr);
                return mpc;
            }
        };

        GRBEnv env;
#ifdef NDEBUG
        env.set(GRB_IntParam_OutputFlag, 0);
#endif
        env.start();
        GRBModel model(env);

        NiceVisitor visitorBasic(&instance, &model);
        NiceVisitor finishedVisitor = td.niceVisit(visitorBasic, true);
        model.optimize();
        auto objVal = model.getObjective().getValue();
        cout << "objective value: " << objVal << endl;

        auto rec = finishedVisitor.reconstruct();
        return rec.toLC(&instance);
    }
}
