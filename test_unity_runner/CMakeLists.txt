cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS
    "../main"
    "../components"
    "${CMAKE_CURRENT_LIST_DIR}/test_sources"
)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(unity_test_runner)

