cmake_minimum_required(VERSION 3.20.2)

option(
    HLEF_ALWAYS_DUMP_CONTEXT 
    "If ON, hlef will always dump the context to stdout"
    OFF)

add_library(hlef
    SHARED
        src/main.c
        src/hlef_scan.c
        src/hlef_context.c
        src/hlef_vector.c
        src/hlef_patch.c
        src/hlef_interfaces.c
        src/hlef_hooks.c
        src/hlef.c)
target_compile_definitions(hlef
    PRIVATE
        HLEF_EXPORT
        $<$<BOOL:${HLEF_ALWAYS_DUMP_CONTEXT}>:HLEF_DUMP_CONTEXT>)
target_include_directories(hlef
    PUBLIC 
        include
    PRIVATE
        src/include)
target_link_libraries(hlef
    PRIVATE
        dbghelp
        psapi
        blam
        $<$<C_COMPILER_ID:GNU,CLANG>:-static-libgcc>)
target_compile_features(hlef
    PUBLIC
        c_std_11)
set_target_properties(hlef
    PROPERTIES
        PREFIX ""
        ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
