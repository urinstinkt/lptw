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

set(GUROBI_LIB $ENV{GUROBI_HOME}/lib)
set(GUROBI_INCLUDE $ENV{GUROBI_HOME}/include)

add_executable(LinePlanning main.cpp Graph.cpp DataParser.cpp TreeSolver.cpp LinePlanning.cpp TreeDecomposition.cpp SpecializedTD.cpp Solver.cpp PathPattern.cpp Graphics.cpp)
add_executable(LP_TD main_TD_util.cpp Graph.cpp DataParser.cpp LinePlanning.cpp TreeDecomposition.cpp SpecializedTD.cpp)
add_executable(RingTDExperiment TD_ring_experiment.cpp Graph.cpp DataParser.cpp LinePlanning.cpp TreeDecomposition.cpp SpecializedTD.cpp)
add_executable(LP_TW2ILP TW2ILP/main.cpp TW2ILP/Solver.cpp Graph.cpp DataParser.cpp LinePlanning.cpp TreeDecomposition.cpp SpecializedTD.cpp Graphics.cpp)

target_include_directories(LP_TW2ILP PUBLIC ${GUROBI_INCLUDE})
target_link_libraries(LP_TW2ILP ${GUROBI_LIB}/gurobi100.lib)
target_link_libraries(LP_TW2ILP debug ${GUROBI_LIB}/gurobi_c++mdd2017.lib optimized ${GUROBI_LIB}/gurobi_c++md2017.lib)

add_dependencies(LinePlanning tree_decomp)
add_dependencies(LP_TD tree_decomp)
add_dependencies(LP_TW2ILP tree_decomp)