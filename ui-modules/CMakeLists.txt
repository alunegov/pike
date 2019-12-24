cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(modules LANGUAGES CXX)

set(source_list
    "main/MainPresenter.h"
    "main/MainPresenterImpl.cpp"
    "main/MainPresenterImpl.h"
    "main/MainView.h"
)

add_library(${PROJECT_NAME} ${source_list})

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)

target_include_directories(${PROJECT_NAME}
    PUBLIC "main"
)

if(MSVC)
    target_compile_definitions(${PROJECT_NAME}
        PRIVATE _CRT_SECURE_NO_WARNINGS
    )
endif()

if(MSVC)
    target_compile_options(${PROJECT_NAME}
        PRIVATE /W4
    )
else()
    target_compile_options(${PROJECT_NAME}
        PRIVATE -Wall -Wextra
    )
endif()

target_link_libraries(${PROJECT_NAME}
    PUBLIC logic
)
