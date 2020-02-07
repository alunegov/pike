#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <system_error>
#include <vector>

#include <tl/expected.hpp>

namespace ros { namespace dc {

// Плата АЦП/ЦАП/ТТЛ
class DAQ
{
public:
    using _Channels = std::vector<uint16_t>;
    using AdcReadCallback = void(const int16_t* values, size_t values_count);

    virtual ~DAQ() = default;

    virtual tl::expected<void, std::error_code> Init(size_t slot_num) = 0;

    virtual tl::expected<void, std::error_code> Deinit() = 0;

    virtual tl::expected<void, std::error_code> TtlEnable(bool enable) = 0;

    virtual tl::expected<void, std::error_code> TtlOut(uint16_t value) = 0;

    virtual tl::expected<void, std::error_code> TtlOut_SetPin(uint16_t value) = 0;

    virtual tl::expected<void, std::error_code> TtlOut_ClrPin(uint16_t value) = 0;

    virtual tl::expected<uint16_t, std::error_code> TtlIn() = 0;

    // TODO: нужна отмена долгого чтения
    virtual tl::expected<void, std::error_code> AdcRead(double_t& reg_freq, size_t points_count,
            const _Channels& channels, int16_t* values) = 0;

    virtual tl::expected<void, std::error_code> AdcRead(double_t& reg_freq, const _Channels& channels,
            const std::function<AdcReadCallback>& callback, const std::chrono::milliseconds& callback_interval,
            const std::atomic_bool& cancel_token) = 0;
};

}}
