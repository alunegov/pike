cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(pike-qwidgets LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOUIC ON)
find_package(Qt5 REQUIRED COMPONENTS Widgets Test)

set(source_list
    "main.cpp"
    "_widgets/CameraWidget.cpp"
    "_widgets/CameraWidget.h"
    "_widgets/DistanceWidget.cpp"
    "_widgets/DistanceWidget.h"
    "_widgets/InclioWidget.cpp"
    "_widgets/InclioWidget.h"
    "_widgets/SliceWidget.cpp"
    "_widgets/SliceWidget.h"
    "main/MainViewImpl.cpp"
    "main/MainViewImpl.h"
)

add_executable(${PROJECT_NAME} WIN32 ${source_list})

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)

target_include_directories(${PROJECT_NAME}
    #PRIVATE "."
    PRIVATE "_widgets"
    PRIVATE "main"

    PRIVATE "../devices/cd22"
    PRIVATE "../devices/ender"
    PRIVATE "../devices/inclinometer"
    PRIVATE "../devices/mover"
    PRIVATE "../devices/odometer"
    PRIVATE "../devices/pike"
    PRIVATE "../devices/rotator"
    PRIVATE "../_ext/serial"

    PRIVATE "../dc/lcard"
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

target_link_libraries(${PROJECT_NAME}
    PRIVATE Qt5::Widgets
    PRIVATE modules
)

if(BUILD_TESTING)
    #find_package(Qt5Test)

    add_executable(${PROJECT_NAME}-test
        "_widgets/CameraWidget.cpp"
        "_widgets/DistanceWidget.cpp"
        "_widgets/InclioWidget.cpp"
        "_widgets/SliceWidget.cpp"
        "main/MainViewImpl.cpp"
        "main/MainViewImplTest.cpp"
    )
    target_include_directories(${PROJECT_NAME}-test
        PRIVATE "_widgets"
        PRIVATE "main"

        PRIVATE "../_ext/FakeIt/single_header"
    )
    target_link_libraries(${PROJECT_NAME}-test
        PRIVATE Qt5::Widgets
        PRIVATE Qt5::Test
        PRIVATE modules
    )
    add_test(${PROJECT_NAME}-test ${PROJECT_NAME}-test)
endif()
