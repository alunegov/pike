#pragma once

#include <cmath>
#include <cstdint>
#include <string>

namespace ros { namespace pike { namespace logic {

// Настройки
// Нумерация каналов АЦП/ТТЛ с 1.
struct Conf
{
    // Длина объекта измерения, мм
    double_t object_length;

    // Настройки платы сбора данных Л-Кард
    struct
    {
        // Номер слота, нумерация с 0
        size_t slot;
        // Флаг: Общая земля у каналов АЦП (иначе дифф. подключение)
        bool common_gnd;
        // Частота регистрации АЦП, кГц
        double_t adc_rate;
    } daq;

    // Настройки Датчика глубины CD22
    struct
    {
        // Имя COM-порта, в формате COM1 или \\.\COM43
        std::string port_name;
        // Скорость связи, бод
        int32_t baud_rate;
    } depthometer;

    // Настройки К1
    struct
    {
        // Номер входной линии ТТЛ
        uint16_t pin;
    } ender1;

    // Настройки К2
    struct
    {
        // Номер входной линии ТТЛ
        uint16_t pin;
    } ender2;

    // Настройки Инклинометра
    struct
    {
        // Номер канала АЦП для канала X
        uint16_t x_channel;
        // Номер канала АЦП для канала Y
        uint16_t y_channel;
        // Имя файла с таблицей пересчёта в градусы
        std::string trans_table_file;
    } inclinometer;

    // Настройки Коллекторного двигателя
    struct
    {
        // Номер выходной линии ТТЛ для PWM
        uint16_t pwm_pin;
        // Номер выходной линии ТТЛ для Dir
        uint16_t dir_pin;
    } mover;

    // Настройки Датчика расстояния ЛИР
    struct
    {
        // Номер канала АЦП для канала A
        uint16_t a_channel;
        // Номер канала АЦП для канала B
        uint16_t b_channel;
        // Уровень для детектирования "пульсов", В
        double_t threshold;
        // Дистанция за один "пульс", мм
        double_t distance_per_pulse;
    } odometer;

    // Настройки Шагового двигателя
    struct
    {
        // Номер вsходной линии TTL для En
        uint16_t en_pin;
        // Номер вsходной линии TTL для Step
        uint16_t step_pin;
        // Номер вsходной линии TTL для Dir
        uint16_t dir_pin;
        // Номер вsходной линии TTL для M0
        uint16_t mx_pin;
        // Количество шагов в полном обороте в режиме измерения (M0 = 0)
        uint32_t steps_per_msr;
        // Количество шагов в полном обороте в режиме ручного вращения (M0 = 1)
        uint32_t steps_per_view;
    } rotator;

    // Настройки сервера "удалённого" управления движением
    struct
    {
        // Номер UDP-порта для приёма сообщений от "удалённого" джойстика
        uint16_t port;
    } remote;
};

}}}
