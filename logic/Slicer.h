#pragma once

#include <Pike.h>

namespace ros { namespace pike { namespace logic {

// Измерение сечения
class Slicer {
public:
    Slicer() = delete;

    explicit Slicer(ros::devices::Pike* pike) :
        pike_{pike}
    {}

    void Slice();

private:
    ros::devices::Pike* pike_{nullptr};
};

}}}
