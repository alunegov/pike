#pragma once

#include <cstdint>
#include <functional>
#include <thread>

#include <Pike.h>

namespace ros { namespace pike { namespace logic {

class MovementAndOrientationReader {
public:
    using CallbackFunc = void(int32_t, double_t);

    MovementAndOrientationReader() = delete;

    explicit MovementAndOrientationReader(ros::devices::Pike* pike) :
        pike_{pike}
    {}

    ~MovementAndOrientationReader();

    void Start(std::function<CallbackFunc> callback);

private:
    ros::devices::Pike* pike_{nullptr};

    std::thread thread_;
    std::atomic_bool cancel_token_{false};
};

}}}
