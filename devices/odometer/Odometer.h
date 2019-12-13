#pragma once

#include <atomic>
#include <cstdint>
#include <vector>

namespace ros { namespace devices {

// Одометр/курвиметр/датчик пройденного расстояния (ЛИР-119)
class Odometer {
public:
    Odometer() = delete;

    Odometer(uint16_t a_channel, uint16_t b_channel) :
        a_channel_{a_channel},
        b_channel_{b_channel}
    {}

    void FillChannels(std::vector<uint16_t>& channels);

    void Update(const std::vector<uint16_t>& channels, const std::vector<int16_t>& values, double_t adc_to_volt);

    int32_t Get();

    void Reset();

private:
    uint16_t a_channel_{0};
    uint16_t b_channel_{0};

    std::atomic_int32_t distance_{0};
};

}}
