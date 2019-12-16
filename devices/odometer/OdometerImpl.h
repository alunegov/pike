#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <vector>

#include <Odometer.h>

namespace ros { namespace devices {

// Одометр/курвиметр/датчик пройденного расстояния (ЛИР-119)
class OdometerImpl : public Odometer {
public:
    OdometerImpl() = delete;

    OdometerImpl(uint16_t a_channel, uint16_t b_channel) :
        a_channel_{a_channel},
        b_channel_{b_channel}
    {}

    void FillChannels(std::vector<uint16_t>& channels) override;

    void Update(const std::vector<uint16_t>& channels, const std::vector<int16_t>& values, double_t adc_to_volt) override;

    int32_t Get() override;

    void Reset() override;

private:
    uint16_t a_channel_{0};
    uint16_t b_channel_{0};

    std::atomic_int32_t distance_{0};
};

}}
