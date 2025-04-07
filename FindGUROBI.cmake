

find_path(GUROBI_INCLUDE_DIRS
    NAMES gurobi_c.h
    HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME}
    PATH_SUFFIXES include)

find_library(GUROBI_LIBRARY
    NAMES gurobi gurobi110 gurobi120
    HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME}
    PATH_SUFFIXES lib)

if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    find_library(GUROBI_CXX_LIBRARY
            NAMES gurobi_g++5.2
            HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME}
            PATH_SUFFIXES lib)
    set(GUROBI_CXX_DEBUG_LIBRARY ${GUROBI_CXX_LIBRARY})
elseif(MSVC)
    set(MSVC_YEAR "2017")

    if(MT)
        set(M_FLAG "mt")
    else()
        set(M_FLAG "md")
    endif()

    find_library(GUROBI_CXX_LIBRARY
        NAMES gurobi_c++${M_FLAG}${MSVC_YEAR}
        HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME}
        PATH_SUFFIXES lib)
    find_library(GUROBI_CXX_DEBUG_LIBRARY
        NAMES gurobi_c++${M_FLAG}d${MSVC_YEAR}
        HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME}
        PATH_SUFFIXES lib)
else()
    find_library(GUROBI_CXX_LIBRARY
        NAMES gurobi_c++
        HINTS ${GUROBI_DIR} $ENV{GUROBI_HOME}
        PATH_SUFFIXES lib)
    set(GUROBI_CXX_DEBUG_LIBRARY ${GUROBI_CXX_LIBRARY})
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GUROBI DEFAULT_MSG GUROBI_LIBRARY)
