

set(tree_decomp_classfile "${CMAKE_CURRENT_SOURCE_DIR}/tw/exact/MainDecomposer.class")
add_custom_command(
        OUTPUT ${tree_decomp_classfile}
        COMMAND javac "${CMAKE_CURRENT_SOURCE_DIR}/tw/exact/*.java")

add_custom_target(
        tree_decomp
        DEPENDS ${tree_decomp_classfile})