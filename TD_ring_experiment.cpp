#include <iostream>
#include <filesystem>
#include "DataParser.h"
#include "TreeDecomposition.h"

using namespace LinePlanning;
using namespace std;
using filesystem::path;



int main(int argc, char** argv) {

    for (unsigned int ringe = 1; ringe <= 6; ringe++) {
        for (unsigned int speichen = 2; speichen <= 13; speichen ++) {
            TreeDecomposition::EdgeListGraph graph(ringe*speichen+1);
            for (unsigned int r = 0; r < ringe; r++) {
                for (unsigned int s = 0; s < speichen; s++) {
                    graph.addEdge({s*ringe+r, ((s+1)%speichen)*ringe+r});
                    if (r+1 < ringe) {
                        graph.addEdge({s*ringe+r, s*ringe+r+1});
                    }
                    if (r == 0) {
                        graph.addEdge({s*ringe+r, ringe*speichen});
                    }
                }
            }

            auto td = TreeDecomposition::compute(graph);
            cout << "ringe: " << ringe << " speichen: " << speichen << " baumweite: " << td.getTreeWidth() << endl;
        }
    }

    return 0;
}
