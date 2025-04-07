#include <iostream>
#include <filesystem>
#include "DataParser.h"
#include "TreeSolver.h"
#include "TreeDecomposition.h"
#include "Solver.h"

using namespace LinePlanning;
using namespace std;
using filesystem::path;

void solve_tree_instance(path instance_base_folder)
{
    path instance_input_folder = instance_base_folder / "basis";
    path instance_output_folder = instance_base_folder / "line-planning";

    Instance instance = parseInstanceFiles(instance_input_folder);
    LineConceptEx lc = TreeSolver::solve(instance);

    filesystem::create_directory(instance_output_folder);
    outputLineConcept(instance_output_folder / "Line-Concept.lin", lc);
}

void do_tree_decomposition(path instance_base_folder, bool outputGraphics = true)
{
    path instance_input_folder = instance_base_folder / "basis";
    path instance_output_folder = instance_base_folder / "line-planning";
    path instance_graphics_folder = instance_base_folder / "graphics";

    Instance instance = parseInstanceFiles(instance_input_folder);

    filesystem::create_directory(instance_output_folder);
    auto td = TreeDecomposition::compute(TreeDecomposition::convert(instance.graph), instance_output_folder / "out.td");
    cout << "treewidth: " << td.getTreeWidth() << endl;
    if (outputGraphics)
    {
        ofstream dotfile(instance_graphics_folder / "td.dot");
        td.toGraphViz(dotfile);
    }
}

auto solve(path instance_base_folder, int fmax_clamp = -1)
{
    path instance_input_folder = instance_base_folder / "basis";
    path instance_output_folder = instance_base_folder / "line-planning";
    path instance_graphics_folder = instance_base_folder / "graphics";

    Instance instance = parseInstanceFiles(instance_input_folder);
    if (fmax_clamp > 0)
    {
        cout << "clamping f_max to " << fmax_clamp << endl;
        for (auto &[ei, edge] : instance.graph.edges)
        {
            edge->weight.f_max = std::min(edge->weight.f_max, (unsigned int)fmax_clamp);
            edge->weight.f_min = std::min(edge->weight.f_min, edge->weight.f_max);
        }
    }
    if (!filesystem::exists(instance_output_folder / "out.td"))
    {
        TreeDecomposition::compute(TreeDecomposition::convert(instance.graph), instance_output_folder / "out.td");
    }
    else
    {
        cout << "using existing out.td" << endl;
    }
    ifstream tdfile(instance_output_folder / "out.td");
    auto td = TreeDecomposition::parse(tdfile);
    auto res = Solver::solve(instance, td);
    cout << "cost: " << res->cost << endl;
    auto lineConcept1 = res->reconstructLineConcept();
    auto lineConcept2 = lineConcept1.toLineConcept(&instance);
    outputLineConcept(instance_output_folder / "Line-Concept.lin", lineConcept2);
    cout << "output file: Line-Concept.lin" << endl;
    return lineConcept2;
}

#include <random>
#include "Graphics.h"

int main() {

    using namespace Solver;

    //star graph
    /* auto gen = std::random_device();
     Instance instance;
     instance.c_fix = 1;
     int zacken = 5;
     instance.graph.getOrCreateNode(1);
     for (int i = 0; i < zacken; i++)
     {
         instance.graph.getOrCreateNode(2+i);
     }
     for (int i = 2; i <= instance.graph.nodeCount(); i++)
     {
         instance.graph.addEdge(i, 1, i);
     }
 //    for (int i = 0; i < zacken; i++)
 //    {
 //        int f = std::uniform_int_distribution<int>(1,zacken)(gen);
 //        instance.graph.getEdge(2+i)->weight = EdgeInfoEx(f,f);
 //    }
     instance.graph.getEdge(2)->weight = EdgeInfoEx(2,2);
     instance.graph.getEdge(3)->weight = EdgeInfoEx(5,5);
     instance.graph.getEdge(4)->weight = EdgeInfoEx(3,3);
     instance.graph.getEdge(5)->weight = EdgeInfoEx(3,3);
     instance.graph.getEdge(6)->weight = EdgeInfoEx(5,5);*/

    //path graph
    /*Instance instance;
    instance.c_fix = 1;
    instance.graph.getOrCreateNode(1);
    instance.graph.getOrCreateNode(2);
    instance.graph.getOrCreateNode(3);
    instance.graph.getOrCreateNode(4);
    for (int i = 1; i < instance.graph.nodeCount(); i++)
    {
        instance.graph.addEdge(i, i, i+1);
    }
    instance.graph.getEdge(1)->weight = EdgeInfoEx(3,5);
    instance.graph.getEdge(2)->weight = EdgeInfoEx(1,1);
    instance.graph.getEdge(3)->weight = EdgeInfoEx(2,5);*/

    /*Instance instance = parseInstanceFiles("../Data/toy-easy/basis");

    TreeDecomposition::TreeDecomposition td;
    ifstream tdfile("tw-out.td");
    if (tdfile)
    {
        cout << "using cached tree decomposition" << endl;
        td = TreeDecomposition::parse(tdfile);
    }
    else
        td = TreeDecomposition::compute(TreeDecomposition::convert(instance.graph));
    td.toGraphViz(cout);

    auto res = Solver::solve(instance, td);
    cout << res->cost << endl;
    auto lineConcept1 = res->reconstructLineConcept();
    auto lineConcept2 = lineConcept1.toLineConcept(&instance);
    outputLineConcept(string("test.lin"), lineConcept2);
    return 0;*/

    auto instanceFolder = path("../Data/toy");
    auto lc = solve(instanceFolder, 4);
    Graphics::drawInstanceWithLineConcept(instanceFolder, cout);

    /*path instances_dir = "../Data";

    filesystem::directory_iterator dir(instances_dir);
    for (auto f : dir)
    {
        if (f.is_directory())
        {
            if ((f.path().filename().string()).find("ring") != string::npos)
            {
                cout << "skipping " << f << endl;
                continue;
            }
            //cout << "computing td: " << f << endl;
            //do_tree_decomposition(f);

            cout << "solving: " << f << endl;
            solve(f);
        }
    }*/


    return 0;
}
