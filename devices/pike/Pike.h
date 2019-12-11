#pragma once

#include <cstdint>

#include <CD22.h>
#include <Ender.h>
#include <Inclinometer.h>
#include <LCardDevice.h>
#include <Mover.h>
#include <Odometer.h>
#include <Rotator.h>

namespace ros { namespace devices {

// "Пика"
class Pike {
public:
    Pike() = delete;

    Pike(ros::dc::lcard::LCardDevice* daq, ros::devices::Ender* ender1, ros::devices::Ender* ender2,
        ros::devices::Rotator* rotator, ros::devices::Mover* mover, ros::devices::Odometer* odometer,
        ros::devices::Inclinometer* inclinometer, ros::devices::CD22* depthometer) :
        daq_{daq},
        ender1_{ender1},
        ender2_{ender2},
        rotator_{rotator},
        mover_{mover},
        odometer_{odometer},
        inclinometer_{inclinometer},
        depthometer_{depthometer}
    {}

    ~Pike();

    ros::dc::lcard::LCardDevice* daq() const { return daq_; }
    ros::devices::Ender* ender1() const { return ender1_; }
    ros::devices::Ender* ender2() const { return ender2_; }
    ros::devices::Rotator* rotator() const { return rotator_; }
    ros::devices::Mover* mover() const { return mover_; }
    ros::devices::Odometer* odometer() const { return odometer_; }
    ros::devices::Inclinometer* inclinometer() const { return inclinometer_; }
    ros::devices::CD22* depthometer() const { return depthometer_; }

private:
    ros::dc::lcard::LCardDevice* daq_{nullptr};
    ros::devices::Ender* ender1_{nullptr};
    ros::devices::Ender* ender2_{nullptr};
    ros::devices::Rotator* rotator_{nullptr};
    ros::devices::Mover* mover_{nullptr};
    ros::devices::Odometer* odometer_{nullptr};
    ros::devices::Inclinometer* inclinometer_{nullptr};
    ros::devices::CD22* depthometer_{nullptr};
};

}}
