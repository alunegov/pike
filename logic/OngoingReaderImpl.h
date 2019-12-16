#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <functional>
#include <thread>

#include <OngoingReader.h>

#include <Pike.h>

namespace ros { namespace pike { namespace logic {

// Режим измерения пройденного расстояния, положения в пространстве и глубины
class OngoingReaderImpl : public OngoingReader {
public:
    OngoingReaderImpl() = delete;

    explicit OngoingReaderImpl(ros::devices::Pike* pike, double_t adc_rate) :
        pike_{pike},
        adc_rate_{adc_rate}
    {}

    ~OngoingReaderImpl() override;

    void Start(const std::function<CallbackFunc>& callback) override;

    void IdleDepth(bool value) override;

private:
    ros::devices::Pike* pike_{nullptr};

    double_t adc_rate_{0};

    std::thread thread_;
    std::atomic_bool cancel_token_{false};

    std::atomic_bool depth_idle_token_{false};
};

}}}