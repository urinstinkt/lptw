
#include "DataParser.h"

#include <map>
#include <fstream>
#include <sstream>
#include <filesystem>
using namespace std;

template <class Consumer>
void readCSV(istream &stream, char delimeter, Consumer consumer)
{
    vector<string> fields = {""};
    int c;
    bool skipLine = false;
    while (true)
    {
        c = stream.get();
        if (c == '\n' || c == EOF)
        {
            if (fields.size() > 1 || !fields[0].empty()) //ignore empty lines
                consumer(fields);
            if (c == EOF)
                return;
            fields = {""};
            skipLine = false;
        }
        else if (c == '#')
        {
            skipLine = true;
        }
        else if (isspace(c) || skipLine)
        {
            continue;
        }
        else if (c == delimeter)
        {
            fields.emplace_back();
        }
        else
        {
            fields.back().push_back((char)c);
        }
    }
}

namespace LinePlanning
{
    Instance parseInstanceFiles(const filesystem::path& inputDirectory)
    {
        Instance instance;
        ifstream fileConfig(inputDirectory / "Private-Config.cnf");
        ifstream fileEdges(inputDirectory / "Edge.giv");
        ifstream fileEdgeWeights(inputDirectory / "Load.giv");
        //TODO: handle file failures
        //TODO: handle stoi,stod failures

        unordered_map<string,double> configVals;

        //default config values
        configVals["lpool_costs_fixed"] = 50;
        configVals["lpool_costs_length"] = 0.05;
        configVals["lpool_costs_edges"] = 0.05;

        auto configReader = [&configVals](const vector<string> &values){
            string name = values[0];
            try
            {
                double val = stod(values[1]);
                configVals[name] = val;
            }
            catch (...)
            {}
        };
        readCSV(fileConfig, ';', configReader);

        auto edgeReader = [&instance](const vector<string> &values){
            int edgeId = stoi(values[0]);
            int leftStop = stoi(values[1]);
            int rightStop = stoi(values[2]);
            double length = stod(values[3]);
            try
            {
                auto edge = instance.graph.addEdge(edgeId, leftStop, rightStop);
                edge->weight.length = length;
            }
            catch (const InstanceGraph::DuplicateEdgeException &ex)
            {
            }
        };
        readCSV(fileEdges, ';', edgeReader);

        auto edgeWeightReader = [&instance](const vector<string> &values){
            int edgeId = stoi(values[0]);
            auto edge = instance.graph.getEdge(edgeId);
            if (edge != nullptr)
            {
                edge->weight.f_min = stoi(values[2]);
                edge->weight.f_max = stoi(values[3]);
            }
        };
        readCSV(fileEdgeWeights, ';', edgeWeightReader);

        try
        {
            instance.c_fix = configVals["lpool_costs_fixed"];
            double c_l = configVals["lpool_costs_length"];
            double c_e = configVals["lpool_costs_edges"];
            for (auto it : instance.graph.edges)
            {
                auto edge = it.second;
                edge->weight.cost = edge->weight.length*c_l + c_e*1;
            }
        }
        catch (const out_of_range &ex)
        {
            throw ex;
        }

        return instance;
    }

    std::unordered_map<int, NodeInfoEx> parseNodeInfo(istream& stream){

        std::unordered_map<int, NodeInfoEx> data;

        Instance instance;

        auto reader = [&data](const vector<string> &values){
            int stopId = stoi(values[0]);
            NodeInfoEx &d = data[stopId];
            d.shortName = values[1];
            d.longName = values[2];
            d.x = stod(values[3]);
            d.y = stod(values[4]);
        };
        readCSV(stream, ';', reader);
        return data;
    }

    void outputLineConcept(ostream &out, const LineConcept &lineConcept, bool writeFrequencies) {
        out << "# line-id; edge-order; edge-id; frequency" << endl;
        int line_id = 1;
        for (const Line &line : lineConcept.lines)
        {
            int eo = 1;
            for (Line::Edge edgeId : line.edges)
            {
                out << line_id << "; " << eo << "; " << edgeId;
                if (writeFrequencies) {
                    out << "; " << line.frequency;
                }
                out << endl;
                eo++;
            }
            line_id++;
        }
    }

    void outputLineConcept(const std::filesystem::path &filepath, const LineConcept &lineConcept, bool writeFrequencies)
    {
        ofstream stream(filepath.string());
        if (!stream.good())
        {
            throw std::runtime_error("outputLineConcept: could not open file");
        }
        outputLineConcept(stream, lineConcept);
    }

    void outputPoolCosts(ostream &out, const Instance &instance, const LineConcept &lineConcept) {
        out << "# line-id; length; cost" << endl;
        int line_id = 1;
        for (const Line &line : lineConcept.lines)
        {
            double len = 0;
            double cost = instance.c_fix;
            for (Line::Edge edgeId : line.edges)
            {
                cost += instance.edgeCost(edgeId);
                len += instance.graph.getEdge(edgeId)->weight.length;
            }
            out << line_id << "; " << len << "; " << cost << endl;
            line_id++;
        }
    }

    void outputPoolCosts(const std::filesystem::path &filepath, const Instance &instance, const LineConcept &lineConcept) {
        ofstream stream(filepath.string());
        if (!stream.good())
        {
            throw std::runtime_error("outputPoolCosts: could not open file");
        }
        outputPoolCosts(stream, instance, lineConcept);
    }

    LineConcept parseLineConcept(const filesystem::path &inputFile) {
        LineConcept lc;
        map<int, Line> lines;

        ifstream fileStream(inputFile);
        auto reader = [&lines](const vector<string> &values){
            int lineID = stoi(values[0]);
            auto &l = lines[lineID];
            int eo = stoi(values[1]);
            if (eo-1 >= l.edges.size())
                l.edges.resize(eo);
            l.edges[eo-1] = stoi(values[2]);
            l.frequency = stoi(values[3]);
        };
        readCSV(fileStream, ';', reader);

        for (auto &[i, l] : lines)
        {
            lc.lines.push_back(l);
        }
        return lc;
    }
}
