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

    std::this_thread::sleep_for(TtlOpDelay);

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
    std::this_thread::sleep_for(TtlOpDelay);

    // TODO: return random
    return static_cast<uint16_t>(0);
}

tl::expected<void, std::error_code> DummyDaq::AdcRead(double_t& reg_freq, size_t points_count, const _Channels& channels,
        int16_t* values, const std::function<FiniteAdcReadCallback>& callback, const std::atomic_bool& cancel_token)
{
    (void)values;
    (void)cancel_token;

    assert(reg_freq > 0);
    assert(points_count > 0);
    assert(!channels.empty());
    assert(values != nullptr);

    const auto reg_time{static_cast<std::chrono::milliseconds::rep>(points_count * channels.size() / reg_freq)};
    std::this_thread::sleep_for(std::chrono::milliseconds{reg_time});

    // TODO: fill values with random

    if (callback) {
        callback(points_count * channels.size());
    }

    return {};
}

tl::expected<void, std::error_code> DummyDaq::AdcRead(double_t& reg_freq, const _Channels& channels,
        const std::function<InfiniteAdcReadCallback>& callback, const std::chrono::milliseconds& callback_interval,
        const std::atomic_bool& cancel_token)
{
    assert(reg_freq > 0);
    assert(!channels.empty());
    assert(callback_interval.count() > 0);

    // подбираем размер буфера так, чтобы он наполн€лс€ за callback_interval мс с частотой сбора reg_freq √ц
    auto half_buffer{static_cast<size_t>(reg_freq * callback_interval.count()) * channels.size()};
    // слишком маленький полу-буфер - делаем хот€ бы на кадр
    if (half_buffer == 0) {
        half_buffer = channels.size();
    }

    const std::vector<int16_t> data(2 * half_buffer);

    size_t f1 = 0;

    while (!cancel_token) {
        std::this_thread::sleep_for(callback_interval);

        const int16_t* const data_tmp = data.data() + half_buffer * f1;
        // TODO: fill data_tmp with random
        callback(data_tmp, half_buffer);

        f1 = (f1 == 0) ? 1 : 0;
    }

    return {};
}

tl::expected<void, std::error_code> DummyDaq::DacWrite(uint16_t channel, int16_t value)
{
    (void)channel;
    (void)value;

    return {};
}

}}}
