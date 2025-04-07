

#ifndef LINEPLANNING_GRAPHICS_H
#define LINEPLANNING_GRAPHICS_H

#include "LinePlanning.h"
#include <filesystem>

namespace Graphics {

    using namespace LinePlanning;
    using std::filesystem::path;

    void drawInstanceWithLineConcept(path instanceFolder, LineConcept const& lc, std::ostream& os);
    void drawInstanceWithLineConcept(path instanceFolder);
}


#endif //LINEPLANNING_GRAPHICS_H
