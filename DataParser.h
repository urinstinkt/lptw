
#ifndef LINEPLANNING_DATAPARSER_H
#define LINEPLANNING_DATAPARSER_H

#include "Graph.h"
#include "LinePlanning.h"
#include <string>
#include <fstream>
#include <filesystem>

using std::string;

namespace LinTim {
    class Config;
}

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

    LineConcept parseLineConcept(const std::filesystem::path &inputFile);

    using std::filesystem::path;

    class Config {
        std::shared_ptr<LinTim::Config> config;

    public:
        explicit Config(std::initializer_list<std::filesystem::path> config_path_with_fallbacks);

        double getDouble(string key) const;
        int getInt(string key) const;
        string getString(string key) const;
        bool getBool(string key) const;
    };

    class Project {

    public:
        path rootDirectory;
        path input_folder;
        path output_folder;
        path graphics_folder;
        Config config;

        explicit Project(const std::filesystem::path& rootDirectory);

        Instance parseInstanceFiles() const;

    };
}


#endif //LINEPLANNING_DATAPARSER_H
