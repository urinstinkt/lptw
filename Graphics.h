

#ifndef LINEPLANNING_GRAPHICS_H
#define LINEPLANNING_GRAPHICS_H

#include <filesystem>
#include "LinePlanning.h"
#include "TreeDecomposition.h"
#include "DataParser.h"

namespace Graphics {

    using namespace LinePlanning;
    using std::filesystem::path;

    void drawInstanceWithLineConcept(const Project &project, LineConcept const& lc, std::ostream& os);
    void drawInstanceWithLineConcept(const Project &project);
    void drawTreeDecomposition(const TreeDecomposition::TreeDecomposition &td, path outputFolder);
}


#endif //LINEPLANNING_GRAPHICS_H
