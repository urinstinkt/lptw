#include <iostream>
#include <filesystem>
#include "Solver.h"
#include "../DataParser.h"
#include "../Graphics.h"

using namespace LinePlanning;
using namespace std;
using filesystem::path;

auto solve(path instance_base_folder, Solver::Options options, bool writeLinePool = false)
{
    Project project(instance_base_folder);

    Instance instance = project.parseInstanceFiles();
    if (!filesystem::exists(project.output_folder)) {
        filesystem::create_directory(project.output_folder);
    }
    if (!filesystem::exists(project.output_folder / "out.td"))
    {
        cout << "computing tree decomposition" << endl;
        TreeDecomposition::compute(TreeDecomposition::convert(instance.graph), project.output_folder / "out.td", options.enableSpecializedTD);
    }
    else
    {
        cout << "using existing out.td" << endl;
    }
    ifstream tdfile(project.output_folder / "out.td");
    auto td = TreeDecomposition::parse(tdfile);
    cout << "treewidth: " << td.getLargestBagSize()-1 << endl;

    {
        ofstream td_dotfile(project.graphics_folder / "td.dot");
        td.toGraphViz(td_dotfile);
        string str = "neato -Tpng "+(project.graphics_folder / "td.dot").string()+" -o \""+(project.graphics_folder / "td.png").string()+"\"";
        ::system(str.c_str());
    }

    auto lineConcept = Solver::solve(instance, td, options);
    cout << "feasible: " << lineConcept.isFeasible(instance, true) << endl;
    LineConcept::Costs costs = lineConcept.calcCost(instance);
    cout << "cost: " << costs.costTotal << endl;
    cout << "cost contribution cfix: " << costs.cost_cfix << endl;
    cout << "cost contribution edges: " << costs.cost_edges << endl;
    outputLineConcept(project.output_folder / "Line-Concept.lin", lineConcept);
    cout << "output file: " << project.output_folder / "Line-Concept.lin" << endl;

    if (writeLinePool){
        outputLineConcept(project.input_folder / "Pool.giv", lineConcept, false);
        cout << "output file: " << project.input_folder / "Pool.giv" << endl;
        outputPoolCosts(project.input_folder / "Pool-Cost.giv", instance, lineConcept);
        cout << "output file: " << project.input_folder / "Pool-Cost.giv" << endl;
    }
    return lineConcept;
}

int main(int argc, char** argv) {

    TreeDecomposition::TD_App_Classpath = (path(argv[0]).parent_path().parent_path() / TreeDecomposition::TD_App_Classpath).string();
    //cout << TreeDecomposition::TD_App_Classpath << endl;

    if (argc <= 1)
    {
        path exefile = argv[0];
        cout << "usage: " << exefile.filename().string() << " <input_folder> <optional parameters>" << endl;
        cout << "optional parameters:" << endl;
        cout << "  -t<value>: time limit for ILP solving, in seconds" << endl;
        cout << "  -mg<value>: relative MIP optimality gap (Gurobi MIPGap)" << endl;
        cout << "  -td-default: disable specialized tree decomposition algorithms" << endl;
        //cout << "computes the optimal line concept" << endl;
        cout << "outputs:" << endl;
        cout << "  solution:      <input_folder>/line-planning/Line-Concept.lin" << endl;
        cout << "  visualization: <input_folder>/graphics/line-plan.png" << endl;
        return 0;
    }

    path instance_dir = argv[1];

    Solver::Options options;
    for (int i = 2; i < argc; i++) {
        string par = argv[i];
        if (par == "-td-default") {
            options.enableSpecializedTD = false;
        }
        else if (par.starts_with("-t")) {
            par = par.substr(2);
            options.maxSolveTimeILP = std::stod(par);
        }
        else if (par.starts_with("-mg")) {
            par = par.substr(3);
            options.MIPGap = std::stod(par);
        }
    }

    if (filesystem::is_directory(instance_dir))
    {
        try
        {
            auto lineConcept = solve(instance_dir,options);
            Graphics::drawInstanceWithLineConcept(instance_dir);
            return 0;
        } catch(std::runtime_error &err)
        {
            cerr << err.what() << endl;
            return 1;
        }
    }
    else
    {
        cerr << "error: directory " << instance_dir.string() << " not found" << endl;
        return 1;
    }
}
