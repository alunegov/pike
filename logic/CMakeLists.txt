cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(logic LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOUIC ON)
find_package(Qt5 REQUIRED COMPONENTS Network)

set(source_list
    "Conf.h"
    "ConfMapper.cpp"
    "ConfMapper.h"
    "error_logic.cpp"
    "error_logic.hpp"
    "OngoingReader.h"
    "OngoingReaderImpl.cpp"
    "OngoingReaderImpl.h"
    "Pike.h"
    "PikeImpl.cpp"
    "PikeImpl.h"
    "RemoteServer.h"
    "RemoteServerImpl.cpp"
    "RemoteServerImpl.h"
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

    PRIVATE "../shared"

    PRIVATE "../_ext/rapidjson/include"
)

if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /W4)
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)
endif()

target_link_libraries(${PROJECT_NAME}
    PUBLIC Qt5::Network
    PUBLIC devices
)

if(BUILD_TESTING)
    add_executable(${PROJECT_NAME}-test
        "../_misc/catch2/catch2-main.cpp"
        "ConfMapperTest.cpp"
        "OngoingReaderImplTest.cpp"
        "PikeImplTest.cpp"
        "SlicerImplTest.cpp"
    )

    target_include_directories(${PROJECT_NAME}-test
        PRIVATE "."

        PRIVATE "../_ext/Catch2/single_include"
        # fix for __has_include (FakeIt and VS2015)
        PRIVATE "../_ext/Catch2/single_include/catch2"
        PRIVATE "../_ext/FakeIt/single_header"
    )

    target_link_libraries(${PROJECT_NAME}-test
        PRIVATE ${PROJECT_NAME}
    )

    add_test(${PROJECT_NAME}-test ${PROJECT_NAME}-test)
endif()
