#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <functional>
#include <vector>

namespace ros { namespace dc {

// œÎ‡Ú‡ ¿÷œ/÷¿œ/““À
class DAQ {
public:
    using _Channels = std::vector<uint16_t>;
    using AdcReadCallback = void(const int16_t* values, size_t values_count);

    virtual ~DAQ() = default;

    virtual void Init(size_t slot_num) = 0;

    virtual void Deinit() = 0;

    virtual void TtlEnable(bool enable) = 0;

    virtual void TtlOut(uint16_t value) = 0;

    virtual void TtlOut_SetPin(uint16_t value) = 0;

    virtual void TtlOut_ClrPin(uint16_t value) = 0;

    virtual uint16_t TtlIn() = 0;

    virtual void AdcRead(double_t& reg_freq, size_t point_count, const _Channels& channels, int16_t* values) = 0;

    virtual void AdcRead(double_t& reg_freq, const _Channels& channels, const std::atomic_bool& cancel_token,
            const std::function<AdcReadCallback>& callback) = 0;
};

}}
