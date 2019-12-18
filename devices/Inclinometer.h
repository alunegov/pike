#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

namespace ros { namespace devices {

// Инклинометр
class Inclinometer {
public:
    virtual ~Inclinometer() = default;

    // Заполняет номера каналов для регистрации с АЦП
    virtual void FillChannels(std::vector<uint16_t>& channels) = 0;

    // Обновляет угол по зарегистрированному сигналу с каналов
    virtual void Update(const std::vector<uint16_t>& channels, const int16_t* values, size_t values_count,
            double_t adc_to_volt) = 0;

    // Возвращает угол, °
    virtual double_t Get() = 0;
};

}}
