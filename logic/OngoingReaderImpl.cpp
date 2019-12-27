#include <OngoingReaderImpl.h>

#include <cassert>
#include <chrono>
#include <cstdint>
#include <future>
#include <vector>

namespace ros { namespace pike { namespace logic {

OngoingReaderImpl::~OngoingReaderImpl()
{
    cancel_token_ = true;
    if (adc_thread_.joinable()) {
        adc_thread_.join();
    }
    if (ttl_in_thread_.joinable()) {
        ttl_in_thread_.join();
    }
}

void OngoingReaderImpl::SetOutput(OngoingReaderOutput* output)
{
    assert(!adc_thread_.joinable() && !ttl_in_thread_.joinable());  // можно задавать только в выключенном состоянии
    assert(output != nullptr);
    output_ = output;
}

void OngoingReaderImpl::Start()
{
    assert(!adc_thread_.joinable() && !ttl_in_thread_.joinable());

    cancel_token_ = false;

    adc_thread_ = std::thread{[this]() {
        const double_t AdcToVolt{10.0 / 8000.0};  // TODO: get AdcToVolt from daq

        double_t regFreq{adc_rate_};

        std::vector<uint16_t> channels;
        pike_->inclinometer()->FillChannels(channels);
        pike_->odometer()->FillChannels(channels);

        const auto adc_read_callback = [this, AdcToVolt, &channels](const int16_t* values, size_t values_count) {
            // spawn 3 parallel threads for inclio, distance and depth
            auto inclio_f = std::async(std::launch::async, [this, AdcToVolt, &channels, values, values_count]() -> double_t {
                pike_->inclinometer()->Update(channels, values, values_count, AdcToVolt);
                return pike_->inclinometer()->Get();
            });
            auto distance_f = std::async(std::launch::async, [this, AdcToVolt, &channels, values, values_count]() -> double_t {
                pike_->odometer()->Update(channels, values, values_count, AdcToVolt);
                return pike_->odometer()->Get();
            });
            auto depth_f = std::async(std::launch::async, [this]() -> int16_t {
                if (!depth_idle_token_) {
                    return pike_->depthometer()->Read();
                } else {
                    return INT16_MIN;
                }
            });

            const double_t angle = inclio_f.get();
            const double_t distance = distance_f.get();
            const int16_t depth = depth_f.get();

            output_->AdcTick(distance, angle, depth);
#ifndef NDEBUG
            output_->AdcTick_Values(channels, values, values_count, AdcToVolt);
#endif
        };

        // запуск бесконечного чтения, во время которого будет вызываться adc_read_callback при заполнении половины
        // буфера АЦП
        // TODO: настраивать периодичность вызова callback (сейчас он зависит от типа платы и параметров регистрации -
        // скорость заполнения половины буфера)
        pike_->daq()->AdcRead(regFreq, channels, cancel_token_, adc_read_callback);
    }};

    ttl_in_thread_ = std::thread{[this]() {
        // Задержка между чтениями TtlIn (показания ender)
        constexpr std::chrono::milliseconds TtlInDelay{111};

        while (!cancel_token_) {
            if (!depth_idle_token_) {
                // читаем сразу оба ender (за одно чтение ttl_in)
                pike_->ReadAndUpdateTtlIn();
                const bool ender1 = pike_->ender1()->Get();
                const bool ender2 = pike_->ender2()->Get();

                output_->TtlInTick(ender1, ender2);
            }

            std::this_thread::sleep_for(TtlInDelay);
        }
    }};
}

void OngoingReaderImpl::Stop()
{
    assert(adc_thread_.joinable() && ttl_in_thread_.joinable());

    cancel_token_ = true;
    adc_thread_.join();
    ttl_in_thread_.join();
}

void OngoingReaderImpl::IdleDepth(bool value)
{
    depth_idle_token_ = value;
}

}}}
