#include <OngoingReaderImpl.h>

#include <cassert>
#include <cstdint>
#include <vector>

namespace ros { namespace pike { namespace logic {

OngoingReaderImpl::~OngoingReaderImpl()
{
    if (thread_.joinable()) {
        cancel_token_ = true;
        thread_.join();
    }
}

void OngoingReaderImpl::Start(const std::function<CallbackFunc>& callback)
{
    assert(!thread_.joinable());

    cancel_token_ = false;

    thread_ = std::thread{[this, callback]() {
        const size_t PointsCount{1024};
        const double_t AdcToVolt{10.0 / 8000.0};  // TODO: get AdcToVolt from daq

        std::vector<uint16_t> channels;
        pike_->inclinometer()->FillChannels(channels);
        pike_->odometer()->FillChannels(channels);

        std::vector<int16_t> values(PointsCount * channels.size());

        while (!cancel_token_) {
            double_t regFreq{adc_rate_};

            pike_->daq()->AdcRead(regFreq, PointsCount, channels, values.data());

            pike_->inclinometer()->Update(channels, values, AdcToVolt);
            const double_t angle = pike_->inclinometer()->Get();

            pike_->odometer()->Update(channels, values, AdcToVolt);
            const double_t distance = pike_->odometer()->Get();

            int16_t depth{INT16_MIN};
            if (!depth_idle_token_) {
                depth = pike_->depthometer()->Read();
            }

            callback(distance, angle, depth);
        }
    }};
}

void OngoingReaderImpl::IdleDepth(bool value)
{
    depth_idle_token_ = value;
}

}}}
