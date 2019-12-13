#pragma once

#include <atomic>
#include <cstdint>
#include <functional>

#include <Pike.h>

namespace ros { namespace pike { namespace logic {

// Режим измерения сечения
class Slicer {
public:
    using CallbackFunc = void(double_t, int16_t);

    Slicer() = delete;

    explicit Slicer(ros::devices::Pike* pike) :
        pike_{pike}
    {}

    void Read(const std::atomic_bool& cancel_token, std::function<CallbackFunc> callback);

private:
    ros::devices::Pike* pike_{nullptr};
};

}}}
