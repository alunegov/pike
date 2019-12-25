#include <OngoingReaderImpl.h>

#include <cassert>
#include <cstdint>
#include <vector>

namespace ros { namespace pike { namespace logic {

OngoingReaderImpl::~OngoingReaderImpl()
{
    if (adc_thread_.joinable() || ttl_in_thread_.joinable()) {
        cancel_token_ = true;
        if (adc_thread_.joinable()) {
            adc_thread_.join();
        }
        if (ttl_in_thread_.joinable()) {
            ttl_in_thread_.join();
        }
    }
}

void OngoingReaderImpl::SetOutput(OngoingReaderOutput* output)
{
    assert(!adc_thread_.joinable() && !ttl_in_thread_.joinable());  // можно задавать только в выключенном состо€нии
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
            pike_->inclinometer()->Update(channels, values, values_count, AdcToVolt);
            const double_t angle = pike_->inclinometer()->Get();

            pike_->odometer()->Update(channels, values, values_count, AdcToVolt);
            const double_t distance = pike_->odometer()->Get();

            int16_t depth{INT16_MIN};
            if (!depth_idle_token_) {
                depth = pike_->depthometer()->Read();
            }

            output_->AdcTick(distance, angle, depth);
            output_->AdcTick_Values(channels, values, values_count, AdcToVolt);
        };

        pike_->daq()->AdcRead(regFreq, channels, cancel_token_, adc_read_callback);
    }};

    ttl_in_thread_ = std::thread{[this]() {
        while (!cancel_token_) {
            if (!depth_idle_token_) {
                const bool ender1 = pike_->ender1()->Read();
                const bool ender2 = pike_->ender2()->Read();

                output_->TtlInTick(ender1, ender2);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds{1});
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
