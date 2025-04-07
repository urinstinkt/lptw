
#include "TreeDecomposition.h"
#include "SpecializedTD.h"
#include <fstream>
#include <sstream>

using namespace std;

namespace TreeDecomposition {

    std::string TD_App_Classpath = "PACE2017-TrackA-master";

    void EdgeListGraph::print(ostream &ostream) const{
        const char endl = '\n';
        ostream << "p tw " << vertexCount << " " << edgeCount << endl;
        for (Edge e : edges)
        {
            ostream << e.u << " " << e.v << endl;
        }
    }

    void computeWithPaces(filesystem::path in_file, filesystem::path out_file) {

        auto parent = out_file.parent_path();
        if (parent != "" && !exists(parent)){
            filesystem::create_directory(parent);
        }
        auto str = "java -cp "+TD_App_Classpath+" -Xmx8g -Xms8g -Xss10m tw.exact.MainDecomposer < "+in_file.string()+" > "+out_file.string();
        int result = system(str.c_str());
        if (result != 0)
        {
            remove(out_file);
            throw std::runtime_error("error executing \"tw-exact\"");
        }
        //output check
        ifstream f(out_file);
        std::string line;
        if (!std::getline(f, line))
        {
            remove(out_file);
            throw std::runtime_error("error executing \"tw-exact\": empty output");
        }
        if (line.starts_with("Error"))
        {
            remove(out_file);
            throw std::runtime_error("error executing \"tw-exact\": output containing error");
        }
    }

    TreeDecomposition parse(istream &stream) {
        TreeDecomposition td(0);
        std::string line;
        unordered_map<unsigned int,vector<unsigned int>> adjacencyList;
        while (std::getline(stream, line))
        {
            if (line[0] == 'c')
                continue;
            std::istringstream iss(line);
            if (line[0] == 's')
            {
                string t1,t2;
                unsigned int nBags, bagsize, nVertices;
                iss >> t1 >> t2 >> nBags >> bagsize >> nVertices;
                td.treewidth = bagsize-1;
                td.bags = vector<Bag>(nBags);
            }
            else if (line[0] == 'b')
            {
                string t;
                unsigned int id, v;
                iss >> t >> id;
                while (iss >> v)
                {
                    td.bags[id-1].vertices.insert(v);
                }
            }
            else
            {
                unsigned int v1, v2;
                iss >> v1 >> v2;
                v1--;
                v2--;
                adjacencyList[v1].push_back(v2);
                adjacencyList[v2].push_back(v1);
            }
        }

        vector<unsigned int> worklist;
        worklist.push_back(td.bags.size()-1);
        td.bags.back().parent = &td.bags.back();
        while (!worklist.empty()){
            auto node = worklist.back();
            worklist.pop_back();
            for (auto nb : adjacencyList[node]) {
                if (td.bags[nb].parent == nullptr){
                    td.bags[nb].parent = &td.bags[node];
                    td.bags[node].children.push_back(&td.bags[nb]);
                    worklist.push_back(nb);
                }
            }
        }
        td.bags.back().parent = nullptr;

        return td;
    }

    TreeDecomposition computeWithPaces(const EdgeListGraph &graph, filesystem::path outputFile) {

        class TmpFile {
            filesystem::path file;

        public:
            TmpFile(filesystem::path file) : file(file) {}

            operator filesystem::path() const {
                return file;
            }

            ~TmpFile() {
                remove(file);
            }
        };

        EdgeListGraph g_norm = graph;
        unordered_map<Vertex, Vertex> ren, renInv;
        g_norm.indexNormalization(ren, renInv, 1);
        g_norm.renameVertices(ren);

        //TmpFile tmpFile(outputFile.parent_path() / "tw-in.gr");
        TmpFile tmpFile("tw-in.gr");
        ofstream fA((filesystem::path)tmpFile);
        g_norm.print(fA);
        fA.close();
        computeWithPaces((filesystem::path)tmpFile, outputFile);
        ifstream fB(outputFile);
        TreeDecomposition td = parse(fB);
        fB.close();
        td.renameVertices(renInv);
        ofstream fB2(outputFile);
        td.write(fB2);
        return td;
    }

    TreeDecomposition compute(const EdgeListGraph &graph, filesystem::path outputFile, bool enableSpecializedAlgorithms) {

        if (enableSpecializedAlgorithms) {
            //TODO: verify TD
            try {
                auto td = forGrid(convert(graph));
                std::cout << "special tree decomposition: grid" << std::endl;
                ofstream fB(outputFile);
                td.write(fB);
                return td;
            } catch (WrongGraphClassError &e) {
            }

            try {
                auto td = forRings(convert(graph));
                std::cout << "special tree decomposition: rings" << std::endl;
                ofstream fB(outputFile);
                td.write(fB);
                return td;
            } catch (WrongGraphClassError &e) {
            }
        }

        return computeWithPaces(graph, outputFile);
    }

    unsigned int TreeDecomposition::getLargestBagSize() const{
        return treewidth+1;
    }

    unsigned int TreeDecomposition::getTreeWidth() const{
        return treewidth;
    }

    bool TreeDecomposition::isPath() {
        for (auto &bag : bags)
        {
            if (bag.children.size() > 1)
                return false;
        }
        return true;
    }

    void TreeDecomposition::toGraphViz(std::ostream& os) const{
        os << "graph G {" << endl;
        for (int i = 0; i < bags.size(); i++)
        {
            os << i << "[label=\"";
            for (auto j = bags[i].vertices.begin(); j != bags[i].vertices.end(); j++)
            {
                if (j != bags[i].vertices.begin())
                    os << ",";
                os << *j;
            }
            os << "\"];" << endl;
        }
        for (int i = 0; i < bags.size(); i++)
        {
            for (const Bag* c : bags[i].children)
            {
                int j = std::distance(&bags[0], c);
                os << i << "--" << j << endl;
            }
        }
        os << "}" << endl;
    }

    const Bag *TreeDecomposition::root() const{
        return &bags.back();
    }

    TreeDecomposition::TreeDecomposition(TreeDecomposition &&other) {
        treewidth = other.treewidth;
        bags = std::move(other.bags);
    }

    void TreeDecomposition::operator=(TreeDecomposition &&other) {
        treewidth = other.treewidth;
        bags = std::move(other.bags);
    }

    void TreeDecomposition::renameVertices(const unordered_map <Vertex, Vertex> &renaming) {
        for (Bag &b : bags) {
            set<Vertex> nv;
            for (const Vertex &v : b.vertices) {
                nv.insert(renaming.at(v));
            }
            b.vertices = nv;
        }
    }

    void TreeDecomposition::write(ostream &stream) const{
        auto bagsize = treewidth+1;
        auto nBags = bags.size();
        stream << "s td " << nBags << " " << bagsize << " " << 0 << endl;
        for (int i = 0; i < bags.size(); i++){
            stream << "b " << i+1;
            for (auto v : bags[i].vertices) {
                stream << " " << v;
            }
            stream << endl;
        }
        for (int i = 0; i < bags.size(); i++)
        {
            const Bag* c = bags[i].parent;
            if (c != nullptr) {
                int j = std::distance(&bags[0], c);
                stream << i+1 << " " << j+1 << endl;
            }
        }
    }

    TreeDecomposition::TreeDecomposition(unsigned int bagCount) {
        bags = vector<Bag>(bagCount);
    }

}
