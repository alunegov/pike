#pragma once

#include <cmath>
#include <cstdint>
#include <functional>
#include <thread>

#include <Pike.h>

namespace ros { namespace pike { namespace logic {

// Режим измерения пройденного расстояния, положения в пространстве и глубины
// Измерение идёт постоянно, кроме глубины, которая "приостанавливается" на время измерения сечения (Slicer).
class OngoingReader {
public:
    using CallbackFunc = void(int32_t, double_t, int16_t);

    OngoingReader() = delete;

    explicit OngoingReader(ros::devices::Pike* pike) :
        pike_{pike}
    {}

    ~OngoingReader();

    void Start(const std::function<CallbackFunc>& callback);

    void IdleDepth(bool value);

private:
    ros::devices::Pike* pike_{nullptr};

    std::thread thread_;
    std::atomic_bool cancel_token_{false};

    std::atomic_bool depth_idle_token_{false};
};

}}}
