#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <thread>

#include <OngoingReader.h>

#include <Pike.h>

namespace ros { namespace pike { namespace logic {

// Режим измерения пройденного расстояния, положения в пространстве и глубины
class OngoingReaderImpl : public OngoingReader
{
public:
    OngoingReaderImpl() = delete;

    OngoingReaderImpl(ros::devices::Pike* pike, double_t adc_rate) :
        _pike{pike},
        _adc_rate{adc_rate}
    {}

    ~OngoingReaderImpl() override;

    // OngoingReader

    void SetOutput(OngoingReaderOutput* output) override;

    void Start() override;

    void Stop() override;

    void IdleDepth(bool value) override;

private:
    ros::devices::Pike* _pike{nullptr};

    double_t _adc_rate{0};

    OngoingReaderOutput* _output{nullptr};

    std::thread _adc_gather_thread;
    std::thread _adc_process_thread;
    std::thread _ttl_in_thread;
    std::atomic_bool _cancel_token{false};

    std::atomic_bool _depth_idle_token{false};
};

}}}
