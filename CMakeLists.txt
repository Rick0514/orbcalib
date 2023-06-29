cmake_minimum_required(VERSION 2.8)
project(ORBSLAM3_CALIB)

find_program(CCACHE_PROGRAM ccache)
if (CCACHE_PROGRAM)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
    message("success use ccache")
endif ()

add_subdirectory(orbslam3)
add_subdirectory(calib)