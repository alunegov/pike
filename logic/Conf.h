#pragma once

#include <cmath>
#include <cstdint>
#include <string>

namespace ros { namespace pike { namespace logic {

// Настройки
struct Conf
{
    double_t object_length;

    struct
    {
        size_t slot;
        bool common_gnd;
        double_t adc_rate;
    } daq;

    struct
    {
        std::string port_name;
        int32_t baud_rate;
    } depthometer;

    struct
    {
        uint16_t pin;
    } ender1;

    struct
    {
        uint16_t pin;
    } ender2;

    struct
    {
        uint16_t x_channel;
        uint16_t y_channel;
        std::string trans_table_file;
    } inclinometer;

    struct
    {
        uint16_t pwm_pin;
        uint16_t dir_pin;
    } mover;

    struct
    {
        uint16_t a_channel;
        uint16_t b_channel;
        double_t threshold;
        double_t distance_per_pulse;
    } odometer;

    struct
    {
        uint16_t en_pin;
        uint16_t step_pin;
        uint16_t dir_pin;
        uint16_t mx_pin;
        uint32_t steps_per_msr;
        uint32_t steps_per_view;
    } rotator;
};

}}}
