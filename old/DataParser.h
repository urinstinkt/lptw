
#ifndef LINEPLANNING_DATAPARSER_H
#define LINEPLANNING_DATAPARSER_H

#include "Graph.h"
#include "LinePlanning.h"
#include <string>
#include <fstream>
#include <filesystem>

using std::string;

namespace LinePlanning
{
    struct NodeInfoEx{
        string shortName, longName;
        double x,y;
    };

    std::unordered_map<int, NodeInfoEx> parseNodeInfo(std::istream& stream);

    void outputLineConcept(std::ostream &out, const LineConcept &lineConcept, bool writeFrequencies=true);
    void outputLineConcept(const std::filesystem::path &filepath, const LineConcept &lineConcept, bool writeFrequencies=true);

    void outputPoolCosts(std::ostream &out, const Instance &instance, const LineConcept &lineConcept);
    void outputPoolCosts(const std::filesystem::path &filepath, const Instance &instance, const LineConcept &lineConcept);

    Instance parseInstanceFiles(const std::filesystem::path &inputDirectory);

    LineConcept parseLineConcept(const std::filesystem::path &inputFile);

    using std::filesystem::path;
    class Project {
    public:
        path rootDirectory;
        path input_folder;
        path output_folder;
        path graphics_folder;

        Project(const std::filesystem::path& rootDirectory) : rootDirectory(rootDirectory) {
            input_folder = rootDirectory / "basis";
            output_folder = rootDirectory / "line-planning";
            graphics_folder = rootDirectory / "graphics";
        }
    };
}


#endif //LINEPLANNING_DATAPARSER_H
