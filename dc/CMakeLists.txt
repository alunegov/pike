cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(dc LANGUAGES CXX)

set(source_list
    "DAQ.h"
    "dummy/DummyDaq.cpp"
    "dummy/DummyDaq.h"
    "lcard/error_lcard.cpp"
    "lcard/error_lcard.hpp"
    "lcard/LCardDaq.cpp"
    "lcard/LCardDaq.h"
)

add_library(${PROJECT_NAME} ${source_list})

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)

target_include_directories(${PROJECT_NAME}
    PUBLIC "."
    PRIVATE "dummy"
    PRIVATE "lcard"

    PRIVATE "../shared"

    PUBLIC "../_ext/expected/include"
    PRIVATE "../_ext/lcomp/include"
)

if(MSVC)
    target_compile_definitions(${PROJECT_NAME} PRIVATE _SCL_SECURE_NO_WARNINGS)
endif()

if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /W4)
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)
endif()

if(BUILD_TESTING)
    add_executable(${PROJECT_NAME}-test
        "../_misc/catch2/catch2-main.cpp"
        "lcard/LCardDaqTest.cpp"
    )

    target_include_directories(${PROJECT_NAME}-test
        PRIVATE "."
        PRIVATE "lcard"

        PRIVATE "../_ext/Catch2/single_include"
        PRIVATE "../_ext/lcomp/include"
    )

    target_compile_definitions(${PROJECT_NAME}-test
        PUBLIC ${K3_ADCRAW}
        PUBLIC ${K3_ROSFLOAT}
    )
    if(MSVC)
        target_compile_definitions(${PROJECT_NAME}-test PRIVATE _SCL_SECURE_NO_WARNINGS)
    endif()

    target_link_libraries(${PROJECT_NAME}-test
        PRIVATE ${PROJECT_NAME}
        PRIVATE RosMath
    )

    add_test(${PROJECT_NAME}-test ${PROJECT_NAME}-test)
endif()
