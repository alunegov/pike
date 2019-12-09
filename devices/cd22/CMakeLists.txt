cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(cd22 LANGUAGES CXX)

set(source_list
    "CD22.cpp"
    "CD22.h"
    "../../_ext/serial/ceSerial.cpp"
    "../../_ext/serial/ceSerial.h"
)

add_library(${PROJECT_NAME} ${source_list})

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)

target_include_directories(${PROJECT_NAME}
    PUBLIC "."
    PUBLIC "../../_ext/serial"
)

if(MSVC)
    target_compile_options(${PROJECT_NAME}
        PRIVATE /W4
    )
else()
    target_compile_options(${PROJECT_NAME}
        PRIVATE -Wall -Wextra
    )
endif()
