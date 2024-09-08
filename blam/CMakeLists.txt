cmake_minimum_required(VERSION 3.20.2)

add_library(blam
    STATIC
        src/base.c
        src/collision_bsp.c
        src/math.c)
target_include_directories(blam
    PUBLIC 
        include)
target_compile_features(blam
    PUBLIC
        c_std_11)
