cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(dc LANGUAGES CXX)

set(source_list
    "DAQ.h"
    "lcard/LCardDevice.cpp"
    "lcard/LCardDevice.h"
)

add_library(${PROJECT_NAME} ${source_list})

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)

target_include_directories(${PROJECT_NAME}
    PUBLIC "."
    PRIVATE "lcard"

    PUBLIC "../_ext/expected/include"
    PRIVATE "../_ext/lcomp/include"
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
