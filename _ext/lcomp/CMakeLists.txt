cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(lcomp LANGUAGES CXX)

set(source_list
    #"src/../include/lguids.cpp"
    "src/791.cpp"
    "src/e140.cpp"
    "src/e154.cpp"
    "src/e310.cpp"
    "src/e440.cpp"
    "src/e2010.cpp"
    "src/ldevbase.cpp"
    "src/ldevbase_ex.cpp"
    "src/loade440.cpp"
    "src/loade2010.cpp"
    "src/loadplx.cpp"
    "src/maincomp.cpp"
    "src/plx.cpp"
    "src/salayer.cpp"
)

add_library(${PROJECT_NAME} ${source_list})

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)

target_include_directories(${PROJECT_NAME}
    PUBLIC "include"
)

if(MSVC)
    target_compile_definitions(${PROJECT_NAME}
        PRIVATE WIN64
        PRIVATE _CRT_SECURE_NO_WARNINGS
        PRIVATE _VC80_UPGRADE=0x0600
    )
endif()

if(MSVC)
    target_compile_options(${PROJECT_NAME}
        PRIVATE /W3
    )
else()
    target_compile_options(${PROJECT_NAME}
        PRIVATE -Wall -Wextra
    )
endif()
