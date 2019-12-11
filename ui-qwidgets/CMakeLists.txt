cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(pike-qwidgets LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOUIC ON)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(source_list
    "main.cpp"
    "main/MainViewImpl.cpp"
    "main/MainViewImpl.h"
)

add_executable(${PROJECT_NAME} WIN32 ${source_list})

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)

target_include_directories(${PROJECT_NAME}
    PRIVATE "."
    PRIVATE "main"
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

target_link_libraries(${PROJECT_NAME}
    PRIVATE Qt5::Widgets
    PRIVATE modules
)
