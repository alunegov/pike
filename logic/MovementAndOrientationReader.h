#pragma once

#include <Pike.h>

namespace ros { namespace pike { namespace logic {

class MovementAndOrientationReader {
public:
    MovementAndOrientationReader() = delete;

    explicit MovementAndOrientationReader(ros::devices::Pike* pike) :
        pike_{pike}
    {}

    void Start();

private:
    ros::devices::Pike* pike_{nullptr};
};

}}}
