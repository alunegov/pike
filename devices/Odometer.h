#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

namespace ros { namespace devices {

// Одометр/курвиметр/датчик пройденного расстояния
class Odometer {
public:
    virtual ~Odometer() = default;

    virtual void FillChannels(std::vector<uint16_t>& channels) = 0;

    virtual void Update(const std::vector<uint16_t>& channels, const std::vector<int16_t>& values, double_t adc_to_volt) = 0;

    virtual int32_t Get() = 0;

    virtual void Reset() = 0;
};

}}
