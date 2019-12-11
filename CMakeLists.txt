cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(pike LANGUAGES CXX)

add_subdirectory(dc/lcard dc-lcard)
add_subdirectory(devices)
add_subdirectory(logic)
add_subdirectory(ui-modules)
add_subdirectory(ui-qwidgets)
