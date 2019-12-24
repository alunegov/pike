cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(logic LANGUAGES CXX)

set(source_list
    "Conf.h"
    "ConfMapper.cpp"
    "ConfMapper.h"
    "OngoingReader.h"
    "OngoingReaderImpl.cpp"
    "OngoingReaderImpl.h"
    "SliceMsrMapper.h"
    "SliceMsrMapperImpl.cpp"
    "SliceMsrMapperImpl.h"
    "Slicer.h"
    "SlicerImpl.cpp"
    "SlicerImpl.h"
)

add_library(${PROJECT_NAME} ${source_list})

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)

target_include_directories(${PROJECT_NAME}
    PUBLIC "."
    PRIVATE "../_ext/rapidjson/include"
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
    PUBLIC devices
)
