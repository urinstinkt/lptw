

#include "Graphics.h"
#include "DataParser.h"
#include <iostream>
#include <cmath>
#include <iomanip>

namespace Graphics {

    using namespace LinePlanning;
    using namespace std;

    struct vec2 {
        double x,y;

        vec2() : x(0), y(0) {}
        vec2(double x, double y) : x(x), y(y) {}

        void normalize() {
            double dd = x*x+y*y;
            double d = ::sqrt(dd);
            if (d != 0){
                x /= d;
                y /= d;
            }
        }

        vec2 perpendicular() const {
            return {y,-x};
        }

        void operator+=(const vec2 &other) {
            x += other.x;
            y += other.y;
        }
    };

    void drawInstanceWithLineConcept(path instanceFolder, LineConcept const &lineConcept, std::ostream& os)
    {
        Project project(instanceFolder);
        auto inStream = std::ifstream(project.input_folder / "Stop.giv");
        auto nodeData = parseNodeInfo(inStream);

        /*vec2 center;
        double maxd = 0;
        for (auto & [i, node] : nodeData) {
            center.x += node.x/nodeData.size();
            center.y += node.y/nodeData.size();
        }
        for (auto & [i, node] : nodeData) {
            node.x -= center.x;
            node.y -= center.y;
            maxd = max(maxd, node.x*node.x + node.y*node.y);
        }
        maxd = sqrt(maxd)/400;
        for (auto & [i, node] : nodeData) {
            node.x /= maxd;
            node.y /= maxd;
        }*/

        for (auto & [i, node] : nodeData) {
            node.x /= project.config.getDouble("ptn_draw_conversion_factor");
            node.y /= project.config.getDouble("ptn_draw_conversion_factor");
        }


        Instance instance = project.parseInstanceFiles();
        //std::cout << lineConcept.calcCost(instance) << endl;

        /*double shortestNodeDistance = numeric_limits<double>::infinity();
        for (auto &[ei, edge] : instance.graph.edges)
        {
            int u = edge->leftNode->index;
            int v = edge->rightNode->index;
            double d = pow(nodeData[u].x-nodeData[v].x, 2.0)+pow(nodeData[u].y-nodeData[v].y, 2.0);
            shortestNodeDistance = min(shortestNodeDistance, d);
        }
        shortestNodeDistance = sqrt(shortestNodeDistance);
        double lineOffset = shortestNodeDistance*0.5/(lineConcept.lines.size()+1);*/
        double lineOffset = 3.0;

        os << std::fixed;
        os << "digraph {" << endl;
        os << "outputorder=edgesfirst;" << endl;
        os << "node[style=filled];" << endl;
        os << "edge[dir=none];" << endl;
        for (auto &[ni,node] : instance.graph.nodes)
        {
            os << ni << "[pos=\"" << nodeData[ni].x << "," << nodeData[ni].y << "\"];" << endl;
        }
        /*for (auto &[ei, edge] : instance.graph.edges)
        {
            auto ni1 = edge->leftNode->index;
            auto ni2 = edge->rightNode->index;
            os << ni1 << "->" << ni2;
            //os << "[pos=\"" << nodeData[ni1].x << "," << nodeData[ni1].y << " " << nodeData[ni1].x << "," << nodeData[ni1].y << " "
                //<< nodeData[ni2].x << "," << nodeData[ni2].y << " "
                //<< nodeData[ni2].x << "," << nodeData[ni2].y << "\"];";
            os << endl;
        }*/

        os << "node[width=0, shape=point];" << endl;
        //double hue = 0;
        //double hue_inc = 1;
        double hue_inc = 1.0/lineConcept.lines.size();
        double hue = hue_inc/2;
        double saturation = 1.0;
        double value = 0.8;
        for (int li = 0; li < lineConcept.lines.size(); li++)
        {
            auto &line = lineConcept.lines[li];
            if (line.frequency != 0)
            {
                /*auto vertexPath = line.toVertexPath(&instance);
                for (int j = 0; j < vertexPath.size(); j++)
                {
                    auto ni = vertexPath[j];
                    vec2 normal = {0,0};
                    if (j > 0) {
                        normal += vec2{nodeData[vertexPath[j]].x-nodeData[vertexPath[j-1]].x, nodeData[vertexPath[j]].y-nodeData[vertexPath[j-1]].y}.perpendicular();
                    }
                    if (j+1 < vertexPath.size()) {
                        normal += vec2{nodeData[vertexPath[j+1]].x-nodeData[vertexPath[j]].x, nodeData[vertexPath[j+1]].y-nodeData[vertexPath[j]].y}.perpendicular();
                    }
                    normal.normalize();

                    os << "\"" << ni << "-" << li << "\"" << "[pos=\"" << nodeData[ni].x+normal.x*lineOffset*(li+1) << "," << nodeData[ni].y+normal.y*lineOffset*(li+1) << "\"";
                    os << ",color=\"" << hue << " " << saturation << " " << value << "\"];" << endl;
                }
                for (int j = 0; j+1 < vertexPath.size(); j++)
                {
                    auto ni1 = vertexPath[j];
                    auto ni2 = vertexPath[j+1];
                    os << "\"" << ni1 << "-" << li << "\"";
                    os << "->";
                    os << "\"" << ni2 << "-" << li << "\"";
                    os << "[color=\"" << hue << " " << saturation << " " << value << "\"";
                    os << ",fontcolor=\"" << hue << " " << saturation << " " << value << "\"";
                    if (j == 0)
                    {
                        os << ",label=\"" << line.frequency <<"\"";
                        os << ",arrowtail=tee";
                    }
                    if (j+2 == vertexPath.size())
                    {
                        os << ",arrowhead=tee";
                    }
                    if (j == 0 && j+2 == vertexPath.size()) {
                        os << ",dir=both";
                    } else if (j == 0){
                        os << ",dir=back";
                    } else if (j+2 == vertexPath.size()){
                        os << ",dir=forward";
                    }
                    os << "];" << endl;
                }*/
                for (int j = 0; j < line.edges.size(); j++)
                {
                    auto ni1 = instance.graph.getEdge(line.edges[j])->leftNode->index;
                    auto ni2 = instance.graph.getEdge(line.edges[j])->rightNode->index;
                    os << ni1 << "->" << ni2;
                    os << "[color=\"" << hue << " " << saturation << " " << value << "\"";
                    os << ",fontcolor=\"" << hue << " " << saturation << " " << value << "\"";
                    if (j == 0)
                    {
                        os << ",label=\"" << line.frequency <<"\"";
                    }
                    /*if (j == 0)
                    {
                        os << ",arrowtail=tee";
                    }
                    if (j+2 == vertexPath.size())
                    {
                        os << ",arrowhead=tee";
                    }
                    if (j == 0 && j+2 == vertexPath.size()) {
                        os << ",dir=both";
                    } else if (j == 0){
                        os << ",dir=back";
                    } else if (j+2 == vertexPath.size()){
                        os << ",dir=forward";
                    }*/
                    //os << ",pos=\"" << nodeData[ni1].x << "," << nodeData[ni1].y << " " << nodeData[ni2].x << "," << nodeData[ni2].y << "\"";
                    os << "];" << endl;
                }
            }

            hue += hue_inc;
            /*if (hue >= 1)
            {
                hue_inc /= 2;
                hue = hue_inc/2;
            }*/
        }

        os << "}" << endl;

    }


    void drawInstanceWithLineConcept(path instanceFolder)
    {
        auto lc = parseLineConcept(instanceFolder / "line-planning" / "Line-Concept.lin");
        filesystem::create_directory(instanceFolder / "graphics");
        std::ofstream dotfile(instanceFolder / "graphics" / "line-plan.dot");
        drawInstanceWithLineConcept(instanceFolder, lc, dotfile);
        dotfile.close();

        string str = "neato -n2 -Tpng "+(instanceFolder / "graphics" / "line-plan.dot").string()+" -o \""+(instanceFolder / "graphics" / "line-plan.png").string()+"\"";
        int result = system(str.c_str());
        if (result != 0) {
            throw std::runtime_error("neato failed");
        }
    }
}