cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(pike LANGUAGES CXX)

# Задание дефайнов AdcRaw_t/RosFloat_t и RUS/ENG для RosMath через параметры cmake-скрипта
set(K3_ADCRAW "AdcRaw_t=int16_t")
set(K3_ROSFLOAT "RosFloat_t=double_t")
set(K3_L10N "RUS")

if(BUILD_TESTING)
    enable_testing()
endif()

add_subdirectory(_ext/RosMath RosMath)
add_subdirectory(_ext/dc dc) # тесты dc используют RosMath
add_subdirectory(devices)
add_subdirectory(logic)
add_subdirectory(ui-modules)
add_subdirectory(ui-qwidgets)
