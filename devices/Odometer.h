#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

namespace ros { namespace devices {

// Одометр/курвиметр/датчик пройденного расстояния
class Odometer {
public:
    using _Channels = std::vector<uint16_t>;
    using _Values = std::vector<int16_t>;

    virtual ~Odometer() = default;

    // Заполняет номера каналов для регистрации с АЦП
    virtual void FillChannels(_Channels& channels) = 0;

    // Обновляет пройденное расстояние по зарегистрированному сигналу с каналов
    virtual void Update(const _Channels& channels, const _Values& values, double_t adc_to_volt) = 0;

    // Возвращает пройденное расстояние, мм
    // Размерность задаётся размерностью Conf::odometer::distance_per_pulse.
    virtual double_t Get() = 0;

    // Обнуляет пройденное расстояние
    virtual void Reset() = 0;
};

}}
