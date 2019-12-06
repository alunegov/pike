cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(pike LANGUAGES CXX)

add_subdirectory(dc/lcard dc-lcard)

set(source_list
    "main.cpp"
)

add_executable(${PROJECT_NAME} ${source_list})

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)

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
    dc-lcard
)
