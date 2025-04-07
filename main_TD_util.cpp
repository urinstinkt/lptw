#include <iostream>
#include <filesystem>
#include "DataParser.h"
#include "TreeDecomposition.h"

using namespace LinePlanning;
using namespace std;
using filesystem::path;

void do_tree_decomposition(path instance_base_folder, bool outputGraphics = true)
{
    path instance_input_folder = instance_base_folder / "basis";
    path instance_output_folder = instance_base_folder / "line-planning";
    path instance_graphics_folder = instance_base_folder / "graphics";

    Project project(instance_base_folder);
    Instance instance = project.parseInstanceFiles();

    filesystem::create_directory(instance_output_folder);
    auto td = TreeDecomposition::compute(TreeDecomposition::convert(instance.graph), instance_output_folder / "out.td");
    cout << "treewidth: " << td.getTreeWidth() << endl;
    if (outputGraphics)
    {
        ofstream dotfile(instance_graphics_folder / "td.dot");
        td.toGraphViz(dotfile);
    }
}


int main(int argc, char** argv) {

    if (argc <= 1)
    {
        path exefile = argv[0];
        cout << "usage: " << exefile.filename().string() << " <input_folder>" << endl;
        cout << "computes the tree decomposition on the input folder" << endl;
        cout << "outputs:" << endl;
        cout << "  td file:       <input_folder>/line-planning/out.td" << endl;
        cout << "  graphViz file: <input_folder>/graphics/td.dot" << endl;
        return 0;
    }

    TreeDecomposition::TD_App_Classpath = "../"+TreeDecomposition::TD_App_Classpath;

    path instance_dir = argv[1];

    if (filesystem::is_directory(instance_dir))
    {
        try
        {
            do_tree_decomposition(instance_dir);
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
