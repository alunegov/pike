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

    virtual void FillChannels(_Channels& channels) = 0;

    virtual void Update(const _Channels& channels, const _Values& values, double_t adc_to_volt) = 0;

    virtual double_t Get() = 0;
};

}}
