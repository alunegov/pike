#include <MovementAndOrientationReader.h>

#include <cassert>
#include <cstdint>
#include <vector>

namespace ros { namespace pike { namespace logic {

MovementAndOrientationReader::~MovementAndOrientationReader()
{
    if (thread_.joinable()) {
        cancel_token_ = true;
        thread_.join();
    }
}

void MovementAndOrientationReader::Start(std::function<CallbackFunc> callback)
{
    assert(!thread_.joinable());

    cancel_token_ = false;

    thread_ = std::thread{[this, callback]() {
        std::vector<int16_t> values(1024 * 4);

        while (!cancel_token_) {
            double_t regFreq{12.8};

            pike_->daq()->AdcRead(regFreq, 1024, {1 | 32, 2 | 32, 5 | 32, 6 | 32}, values.data());

            pike_->odometer()->Update(values);
            const auto distance = pike_->odometer()->Get();

            pike_->inclinometer()->Update(values);
            const auto angle = pike_->inclinometer()->Get();

            int16_t depth{INT16_MIN};
            if (!depth_idle_token_) {
                depth = pike_->depthometer()->Read();
            }

            callback(distance, angle, depth);
        }
    }};
}

void MovementAndOrientationReader::IdleDepth(bool value)
{
    depth_idle_token_ = value;
}

}}}
