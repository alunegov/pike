cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(devices LANGUAGES CXX)

set(source_list
    "Depthometer.h"
    "Ender.h"
    "error_devices.cpp"
    "error_devices.hpp"
    "Inclinometer.h"
    "Mover.h"
    "Odometer.h"
    "Rotator.h"
    "cd22/CD22.cpp"
    "cd22/CD22.h"
    "ender/EnderImpl.cpp"
    "ender/EnderImpl.h"
    "inclinometer/InclinometerImpl.cpp"
    "inclinometer/InclinometerImpl.h"
    "inclinometer/InclinometerImplTransTable.h"
    "inclinometer/InclinometerImplTransTableMapper.cpp"
    "inclinometer/InclinometerImplTransTableMapper.h"
    "mover/MoverImpl.cpp"
    "mover/MoverImpl.h"
    "odometer/OdometerImpl.cpp"
    "odometer/OdometerImpl.h"
    "rotator/RotatorImpl.cpp"
    "rotator/RotatorImpl.h"

    "../_ext/serial/ceSerial.cpp"
)

add_library(${PROJECT_NAME} ${source_list})

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)

target_include_directories(${PROJECT_NAME}
    PUBLIC "."
    PRIVATE "cd22"
    PRIVATE "ender"
    PRIVATE "inclinometer"
    PRIVATE "mover"
    PRIVATE "odometer"
    PRIVATE "rotator"

    PRIVATE "../_ext/errors-helper"
    PUBLIC "../_ext/expected/include"
    PRIVATE "../_ext/serial"
)

target_compile_definitions(${PROJECT_NAME}
    PUBLIC ${K3_ADCRAW}
    PUBLIC ${K3_ROSFLOAT}
)

if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /W4)
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)
endif()

target_link_libraries(${PROJECT_NAME}
    PUBLIC RosMath
    PUBLIC dc
)

if(BUILD_TESTING)
    add_executable(${PROJECT_NAME}-test
        "../_misc/catch2/catch2-main.cpp"
        "ender/EnderImplTest.cpp"
        "inclinometer/InclinometerImplTest.cpp"
        "inclinometer/InclinometerImplTransTableMapperTest.cpp"
        "odometer/OdometerImplTest.cpp"
    )

    target_include_directories(${PROJECT_NAME}-test
        PRIVATE "."
        PRIVATE "ender"
        PRIVATE "inclinometer"
        PRIVATE "odometer"

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
