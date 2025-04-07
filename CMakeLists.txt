cmake_minimum_required(VERSION 3.19)
project(LinePlanning)

set(CMAKE_CXX_STANDARD 20)

#add_custom_target(tree_decomp COMMAND make
#        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/PACE2017-TrackA-master
#        )

set(tree_decomp_classfile "${CMAKE_CURRENT_SOURCE_DIR}/PACE2017-TrackA-master/tw/exact/MainDecomposer.class")
add_custom_command(
        OUTPUT ${tree_decomp_classfile}
        COMMAND make
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/PACE2017-TrackA-master)
add_custom_target(
        tree_decomp
        DEPENDS ${tree_decomp_classfile})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR})
find_package(GUROBI REQUIRED)
#message(${GUROBI_CXX_LIBRARY})

#add_executable(LinePlanning main.cpp Graph.cpp DataParser.cpp TreeSolver.cpp LinePlanning.cpp TreeDecomposition.cpp SpecializedTD.cpp Solver.cpp PathPattern.cpp Graphics.cpp)
add_executable(LP_TD main_TD_util.cpp Graph.cpp DataParser.cpp LinePlanning.cpp TreeDecomposition.cpp SpecializedTD.cpp)
add_executable(RingTDExperiment TD_ring_experiment.cpp Graph.cpp DataParser.cpp LinePlanning.cpp TreeDecomposition.cpp SpecializedTD.cpp)
add_executable(LP_TW2ILP TW2ILP/main.cpp TW2ILP/Solver.cpp Graph.cpp DataParser.cpp LinePlanning.cpp TreeDecomposition.cpp SpecializedTD.cpp Graphics.cpp)
add_executable(test test/test.cpp Graph.cpp LinePlanning.cpp)

target_include_directories(LP_TW2ILP PUBLIC ${GUROBI_INCLUDE_DIRS})
target_link_libraries(LP_TW2ILP ${GUROBI_LIBRARY})
target_link_libraries(LP_TW2ILP optimized ${GUROBI_CXX_LIBRARY} debug ${GUROBI_CXX_DEBUG_LIBRARY})

#add_dependencies(LinePlanning tree_decomp)
add_dependencies(LP_TD tree_decomp)
add_dependencies(LP_TW2ILP tree_decomp)