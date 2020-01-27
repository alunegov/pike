#include <DummyDaq.h>

#include <cassert>
#include <chrono>
#include <thread>

namespace ros { namespace dc { namespace dummy {

tl::expected<void, std::error_code> DummyDaq::Init(size_t slot_num)
{
    (void)slot_num;

    return {};
}

tl::expected<void, std::error_code> DummyDaq::Deinit()
{
    return {};
}

tl::expected<void, std::error_code> DummyDaq::TtlEnable(bool enable)
{
    (void)enable;

    return {};
}

tl::expected<void, std::error_code> DummyDaq::TtlOut(uint16_t value)
{
    (void)value;

    std::this_thread::sleep_for(std::chrono::milliseconds{1});

    return {};
}

tl::expected<void, std::error_code> DummyDaq::TtlOut_SetPin(uint16_t value)
{
    (void)value;

    return TtlOut(0);
}

tl::expected<void, std::error_code> DummyDaq::TtlOut_ClrPin(uint16_t value)
{
    (void)value;

    return TtlOut(0);
}

tl::expected<uint16_t, std::error_code> DummyDaq::TtlIn()
{
    std::this_thread::sleep_for(std::chrono::milliseconds{1});

    // TODO: return random
    return static_cast<uint16_t>(0);
}

tl::expected<void, std::error_code> DummyDaq::AdcRead(double_t& reg_freq, size_t point_count,
        const _Channels& channels, int16_t* values)
{
    (void)values;

    assert(reg_freq > 0);
    assert(point_count > 0);
    assert((0 < channels.size()) && (channels.size() <= ULONG_MAX));
    assert(values != nullptr);

    const auto reg_time = std::llround(point_count * channels.size() / reg_freq);
    std::this_thread::sleep_for(std::chrono::milliseconds{reg_time});

    // TODO: fill values with random

    return {};
}

tl::expected<void, std::error_code> DummyDaq::AdcRead(double_t& reg_freq, const _Channels& channels,
        const std::atomic_bool& cancel_token, const std::function<AdcReadCallback>& callback)
{
    assert(reg_freq > 0);
    assert((0 < channels.size()) && (channels.size() <= ULONG_MAX));

    //return tl::make_unexpected(std::make_error_code(std::errc::io_error));

    // половина такого буфера б. заполняться за 640 мс при 12.8 кГц
    const size_t buffer{16384};
    const size_t half_buffer{buffer / 2};

    std::vector<int16_t> data(buffer);

    size_t f1 = 0;

    while (!cancel_token) {
        std::this_thread::sleep_for(std::chrono::milliseconds{666});

        const int16_t* const data_tmp = data.data() + half_buffer * f1;
        // TODO: fill data_tmp with random
        callback(data_tmp, half_buffer);

        f1 = (f1 == 0) ? 1 : 0;
    }

    return {};
}

}}}
