#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

namespace ros { namespace dc {

// œÎ‡Ú‡ ¿÷œ/÷¿œ/““À
class DAQ {
public:
    virtual ~DAQ() = default;

    virtual void Init(size_t slot_num) = 0;

    virtual void Deinit() = 0;

    virtual void TtlEnable(bool enable) = 0;

    virtual void TtlOut(uint16_t value) = 0;

    virtual void TtlOut_SetPin(uint16_t value) = 0;

    virtual void TtlOut_ClrPin(uint16_t value) = 0;

    virtual uint16_t TtlIn() = 0;

    virtual void AdcRead(double_t& reg_freq, size_t point_count, const std::vector<uint16_t>& channels, int16_t* values) = 0;
};

}}
