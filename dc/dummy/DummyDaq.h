#pragma once

#include <cmath>
#include <cstdint>
#include <vector>
#include <utility>

#include <DAQ.h>

namespace ros { namespace dc { namespace dummy {

// Плата-заглушка АЦП/ЦАП/ТТЛ
class DummyDaq : public DAQ
{
public:
    DummyDaq() = default;

    // DAQ

    tl::expected<void, std::error_code> Init(size_t slot_num) override;

    tl::expected<void, std::error_code> Deinit() override;

    tl::expected<void, std::error_code> TtlEnable(bool enable) override;

    tl::expected<void, std::error_code> TtlOut(uint16_t value) override;

    tl::expected<void, std::error_code> TtlOut_SetPin(uint16_t value) override;

    tl::expected<void, std::error_code> TtlOut_ClrPin(uint16_t value) override;

    tl::expected<uint16_t, std::error_code> TtlIn() override;

    tl::expected<void, std::error_code> AdcRead(double_t& reg_freq, size_t point_count, const _Channels& channels,
            int16_t* values) override;

    tl::expected<void, std::error_code> AdcRead(double_t& reg_freq, const _Channels& channels,
            const std::atomic_bool& cancel_token, const std::function<AdcReadCallback>& callback) override;
};

}}}
