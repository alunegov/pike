#include <chrono>
#include <future>
#include <thread>

#include <catch2/catch.hpp>

#include <LCardDaq.h>

class LCardDaqTest
{
public:
    LCardDaqTest(ros::dc::lcard::LCardDaq& daq) :
        _daq{daq}
    {}

    ULONG PrepareAdc(double_t& reg_freq, const ros::dc::DAQ::_Channels& channels, size_t* half_buffer, void** data, ULONG** sync)
    {
        return _daq.PrepareAdc(reg_freq, channels, half_buffer, data, sync);
    }

private:
    ros::dc::lcard::LCardDaq& _daq;
};

TEST_CASE("LCardDaq", "[LCardDaq]") {
    ros::dc::lcard::LCardDaq sut;
    LCardDaqTest sut_test{sut};

    // Init
    const auto init_opt = sut.Init(0);

    REQUIRE(init_opt);

    double_t reg_freq{12.8};
    const ros::dc::DAQ::_Channels channels{0, 1, 2, 3};
    size_t half_buffer{0};
    void* data{nullptr};
    ULONG* sync{nullptr};

    // PrepareAdc
    const auto prepare_adc_res = sut_test.PrepareAdc(reg_freq, channels, &half_buffer, &data, &sync);

    REQUIRE(prepare_adc_res == L_SUCCESS);
    REQUIRE(abs(reg_freq - 12.8) < 0.03);
    REQUIRE(half_buffer > 0);
    REQUIRE(data != nullptr);
    REQUIRE(sync != nullptr);

    // AdcRead
    std::atomic_bool cancel_token{false};

    auto adc_read_f = std::async(std::launch::async, [&sut, &reg_freq, &channels, &cancel_token]() {
        size_t callback_counter{0};
        const auto adc_read_callback = [&callback_counter](const int16_t* values, size_t values_count) {
            callback_counter++;
        };

        const auto adc_read_opt = sut.AdcRead(reg_freq, channels, cancel_token, adc_read_callback);

        return std::make_pair(adc_read_opt, callback_counter);
    });

    std::this_thread::sleep_for(std::chrono::seconds{3});
    cancel_token = true;

    const auto adc_read_res = adc_read_f.get();

    REQUIRE(adc_read_res.first);
    REQUIRE(adc_read_res.second > 0);
    REQUIRE(abs(reg_freq - 12.8) < 0.03);
}
