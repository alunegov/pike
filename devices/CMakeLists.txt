cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(devices LANGUAGES CXX)

set(source_list
    "../_ext/serial/ceSerial.cpp"
    "cd22/CD22.cpp"
    "cd22/CD22.h"
    "ender/Ender.cpp"
    "ender/Ender.h"
    "inclinometer/Inclinometer.cpp"
    "inclinometer/Inclinometer.h"
    "mover/Mover.cpp"
    "mover/Mover.h"
    "odometer/Odometer.cpp"
    "odometer/Odometer.h"
    "pike/Pike.cpp"
    "pike/Pike.h"
    "rotator/Rotator.cpp"
    "rotator/Rotator.h"
)

add_library(${PROJECT_NAME} ${source_list})

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)

target_include_directories(${PROJECT_NAME}
    PUBLIC "../_ext/serial"
    PUBLIC "cd22"
    PUBLIC "ender"
    PUBLIC "inclinometer"
    PUBLIC "mover"
    PUBLIC "odometer"
    PUBLIC "pike"
    PUBLIC "rotator"
)

target_compile_definitions(${PROJECT_NAME}
    PUBLIC ${K3_ADCRAW}
    PUBLIC ${K3_ROSFLOAT}
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
    PUBLIC RosMath
    PUBLIC dc-lcard
)
