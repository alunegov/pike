#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

namespace ros { namespace devices {

// Инклинометр
class Inclinometer {
public:
    using _Channels = std::vector<uint16_t>;
    using _Values = std::vector<int16_t>;

    virtual ~Inclinometer() = default;

    // Заполняет номера каналов для регистрации с АЦП
    virtual void FillChannels(_Channels& channels) = 0;

    // Обновляет угол по зарегистрированному сигналу с каналов
    virtual void Update(const _Channels& channels, const _Values& values, double_t adc_to_volt) = 0;

    // Возвращает угол, °
    virtual double_t Get() = 0;
};

}}
