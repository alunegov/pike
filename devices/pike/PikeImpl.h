#pragma once

#include <cstdint>

#include <Pike.h>

#include <DAQ.h>
#include <Depthometer.h>
#include <Ender.h>
#include <Inclinometer.h>
#include <Mover.h>
#include <Odometer.h>
#include <Rotator.h>

namespace ros { namespace devices {

// "Пика"
class PikeImpl : public Pike
{
public:
    PikeImpl() = delete;

    PikeImpl(ros::dc::DAQ* daq, ros::devices::Ender* ender1, ros::devices::Ender* ender2, ros::devices::Rotator* rotator,
            ros::devices::Mover* mover, ros::devices::Odometer* odometer, ros::devices::Inclinometer* inclinometer,
            ros::devices::Depthometer* depthometer) :
        daq_{daq},
        ender1_{ender1},
        ender2_{ender2},
        rotator_{rotator},
        mover_{mover},
        odometer_{odometer},
        inclinometer_{inclinometer},
        depthometer_{depthometer}
    {}

    ~PikeImpl() override;

    // Pike

    ros::dc::DAQ* daq() const override { return daq_; }

    ros::devices::Ender* ender1() const override { return ender1_; }

    ros::devices::Ender* ender2() const override { return ender2_; }

    ros::devices::Rotator* rotator() const override { return rotator_; }

    ros::devices::Mover* mover() const override { return mover_; }

    ros::devices::Odometer* odometer() const override { return odometer_; }

    ros::devices::Inclinometer* inclinometer() const override { return inclinometer_; }

    ros::devices::Depthometer* depthometer() const override { return depthometer_; }

private:
    ros::dc::DAQ* daq_{nullptr};
    ros::devices::Ender* ender1_{nullptr};
    ros::devices::Ender* ender2_{nullptr};
    ros::devices::Rotator* rotator_{nullptr};
    ros::devices::Mover* mover_{nullptr};
    ros::devices::Odometer* odometer_{nullptr};
    ros::devices::Inclinometer* inclinometer_{nullptr};
    ros::devices::Depthometer* depthometer_{nullptr};
};

}}
