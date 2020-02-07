#include <LCardDaq.h>

#include <chrono>
#include <fstream>
#include <future>
#include <thread>

#include <catch2/catch.hpp>

#include <RosMath.h>

// Дружественный к LCardDaq класс
class LCardDaqTest
{
public:
    LCardDaqTest(ros::dc::lcard::LCardDaq& daq) :
        _daq{daq}
    {}

    ULONG PrepareAdc(double_t& reg_freq, const ros::dc::DAQ::_Channels& channels,
            const std::chrono::milliseconds& tick_interval, size_t& half_buffer, void** data, ULONG** sync)
    {
        return _daq.PrepareAdc(reg_freq, channels, tick_interval, half_buffer, data, sync);
    }

private:
    ros::dc::lcard::LCardDaq& _daq;
};

template <typename T,
    std::enable_if_t<std::is_floating_point<T>::value, int> = 0>
constexpr bool double_equals(T a, T b, double_t eps = 0.01)
{
    return std::abs(a - b) < eps;
}

// экспорт сигнала в файл в формате dsv
template <typename T>
void ExportToDsv(const std::vector<T>& values, double_t dx, const std::string& filename)
{
    std::ofstream file{filename};

    for (size_t i = 0; i < values.size(); ++i) {
        file << i * dx << ";" << values[i] << std::endl;
    }
}

static_assert(sizeof(AdcRaw_t) == sizeof(int16_t), "AdcRaw_t size is not 16 bit");

using Harmonic = std::tuple<double_t, double_t, ptrdiff_t>;

// поиск максимальной гармоники в спектре сигнала
Harmonic FindMaxHarmonic(const std::vector<int16_t>& values, double_t df)
{
    assert(!values.empty());
    assert(df > 0);

    const size_t points_count{values.size()};

    const auto channel_mean = Mean_AdcRaw(values.data(), static_cast<uint32_t>(points_count));

    std::vector<double_t> ampls(points_count);
    std::vector<double_t> phases(points_count);

    if (!Rfft(values.data(), ampls.data(), phases.data(), static_cast<uint32_t>(points_count), channel_mean, false)) {
        return std::make_tuple(0.0, 0.0, -1);
    }

    // дальше работаем не со всем спектром, а только с его первой половиной (он симметричен)
    const size_t spec_points_count{points_count / 2};

    Cmplx_Ampl(ampls.data(), phases.data(), static_cast<uint32_t>(points_count), static_cast<uint32_t>(spec_points_count));

    // zero DC
    ampls[0] = 0.0;

    const auto ampls_end{ampls.begin() + spec_points_count};
    const auto max_ampl_it = std::max_element(ampls.begin(), ampls_end);
    if (max_ampl_it == ampls_end) {
        return std::make_tuple(0.0, 0.0, -1);
    }

    const auto max_ampl_index = std::distance(ampls.begin(), max_ampl_it);

    assert(max_ampl_index >= 0);
    const auto max_ampl_freq = IndexToFrequency(static_cast<uint32_t>(max_ampl_index), df);

    return std::make_tuple(*max_ampl_it, max_ampl_freq, max_ampl_index);
}

// поиск максимальной гармоники в спектре, полученном из сигнала указанного канала (из покадровых данных от АЦП)
Harmonic FindMaxHarmonicInChannel(const std::vector<int16_t>& values, size_t points_count, size_t channels_count,
        size_t channel_index, double_t reg_freq)
{
    assert(values.size() >= points_count * channels_count);
    assert(points_count > 0);
    assert(channels_count > 0);
    assert(channel_index <= channels_count);
    assert(reg_freq > 0);

    std::vector<int16_t> channel_values(points_count);

    ExtractChannelFromAdcFrames(values.data(), channel_values.data(), points_count, channels_count, channel_index);

    const auto dt{1 / (reg_freq * 1000)};
    const auto df{1 / (dt * points_count)};

    //ExportToDsv(channel_values, dt, "");

    return FindMaxHarmonic(channel_values, df);
}

const double_t RefRegFreq{12.8};

const uint16_t AdcCommonGnd{1 << 5};
const ros::dc::DAQ::_Channels RefChannels{0 | AdcCommonGnd, 1 | AdcCommonGnd, 2 | AdcCommonGnd, 3 | AdcCommonGnd};

TEST_CASE("LCardDaq AdcRead", "[LCardDaq]") {
    ros::dc::lcard::LCardDaq sut;

    // Init
    const auto init_opt = sut.Init(0);

    REQUIRE(init_opt);

    SECTION("sync mode") {
        double_t reg_freq{RefRegFreq};
        const size_t points_count{1024};
        std::vector<int16_t> vals(points_count * RefChannels.size());

        const auto adc_read_opt = sut.AdcRead(reg_freq, points_count, RefChannels, vals.data());

        REQUIRE(adc_read_opt);
        CHECK(double_equals(reg_freq, RefRegFreq, 0.03));

        const size_t test_points_count{points_count};
        const auto max_harm = FindMaxHarmonicInChannel(vals, test_points_count, RefChannels.size(), 0, reg_freq);

        REQUIRE(std::get<2>(max_harm) != -1);
        CHECK(double_equals(std::get<1>(max_harm), 262.0, 1.0));
    }

    SECTION("async mode") {
        double_t reg_freq{RefRegFreq};
        std::atomic_bool cancel_token{false};

        size_t callback_counter{0};
        std::vector<int16_t> vals;

        auto adc_read_f = std::async(std::launch::async, [&]() {
            const auto adc_read_callback = [&](const int16_t* values, size_t values_count) {
                callback_counter++;

                vals.insert(vals.end(), values, values + values_count);
            };

            return sut.AdcRead(reg_freq, RefChannels, adc_read_callback, std::chrono::milliseconds{100}, cancel_token);
        });

        std::this_thread::sleep_for(std::chrono::seconds{3});
        cancel_token = true;

        const auto adc_read_opt = adc_read_f.get();

        REQUIRE(adc_read_opt);
        CHECK(double_equals(reg_freq, RefRegFreq, 0.03));
        REQUIRE(callback_counter > 0);
        REQUIRE(vals.size() > 0);

        const size_t test_points_count{1024};
        const auto max_harm = FindMaxHarmonicInChannel(vals, test_points_count, RefChannels.size(), 0, reg_freq);

        REQUIRE(std::get<2>(max_harm) != -1);
        CHECK(double_equals(std::get<1>(max_harm), 262.0, 1.0));
    }

    sut.Deinit();
}

TEST_CASE("LCardDaq PrepareAdc", "[LCardDaq]") {
    ros::dc::lcard::LCardDaq sut;
    LCardDaqTest sut_test{sut};

    // Init
    const auto init_opt = sut.Init(0);

    REQUIRE(init_opt);

    double_t reg_freq{RefRegFreq};
    size_t half_buffer{0};
    void* data{nullptr};
    ULONG* sync{nullptr};

    // PrepareAdc
    const auto prepare_adc_res = sut_test.PrepareAdc(reg_freq, RefChannels, std::chrono::milliseconds{600},
            half_buffer, &data, &sync);

    REQUIRE(prepare_adc_res == L_SUCCESS);
    CHECK(double_equals(reg_freq, RefRegFreq, 0.02 * RefRegFreq));
    CHECK(half_buffer / RefChannels.size() / reg_freq == 600);
    CHECK(data != nullptr);
    CHECK(sync != nullptr);

    sut.Deinit();
}
