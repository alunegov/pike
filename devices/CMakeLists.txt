cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(devices LANGUAGES CXX)

set(source_list
    "Depthometer.h"
    "Ender.h"
    "Inclinometer.h"
    "Mover.h"
    "Odometer.h"
    "Pike.h"
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
    "pike/PikeImpl.cpp"
    "pike/PikeImpl.h"
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
    PRIVATE "pike"
    PRIVATE "rotator"
    PRIVATE "../_ext/serial"
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
    PUBLIC dc
)
