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

namespace ros { namespace pike { namespace logic {

// "Пика"
class PikeImpl : public Pike
{
public:
    PikeImpl() = delete;

    PikeImpl(ros::dc::DAQ* daq, ros::devices::Ender* ender1, ros::devices::Ender* ender2, ros::devices::Rotator* rotator,
            ros::devices::Mover* mover, ros::devices::Odometer* odometer, ros::devices::Inclinometer* inclinometer,
            ros::devices::Depthometer* depthometer) :
        _daq{daq},
        _ender1{ender1},
        _ender2{ender2},
        _rotator{rotator},
        _mover{mover},
        _odometer{odometer},
        _inclinometer{inclinometer},
        _depthometer{depthometer}
    {}

    // Pike

    ros::dc::DAQ* daq() const override { return _daq; }

    ros::devices::Ender* ender1() const override { return _ender1; }

    ros::devices::Ender* ender2() const override { return _ender2; }

    ros::devices::Rotator* rotator() const override { return _rotator; }

    ros::devices::Mover* mover() const override { return _mover; }

    ros::devices::Odometer* odometer() const override { return _odometer; }

    ros::devices::Inclinometer* inclinometer() const override { return _inclinometer; }

    ros::devices::Depthometer* depthometer() const override { return _depthometer; }

    bool InMotion() const override { return _is_moving || _is_rotating; }

    //void SetInMotion(bool in_motion) override { _in_motion = in_motion; }

    bool IsMoving() const override { return _is_moving; }

    void SetIsMoving(bool is_moving) override { _is_moving = is_moving; };

    bool IsRotating() const override { return _is_rotating; };

    void SetIsRotating(bool is_rotating) override { _is_rotating = is_rotating; }

    bool IsSlicing() const override { return _is_slicing; }

    void SetIsSlicing(bool is_slicing) override { _is_slicing = is_slicing; }

    tl::expected<void, std::error_code> ReadAndUpdateTtlIn() const override;

private:
    ros::dc::DAQ* _daq{nullptr};
    ros::devices::Ender* _ender1{nullptr};
    ros::devices::Ender* _ender2{nullptr};
    ros::devices::Rotator* _rotator{nullptr};
    ros::devices::Mover* _mover{nullptr};
    ros::devices::Odometer* _odometer{nullptr};
    ros::devices::Inclinometer* _inclinometer{nullptr};
    ros::devices::Depthometer* _depthometer{nullptr};
    //bool _in_motion{false};
    bool _is_moving{false};
    bool _is_rotating{false};
    bool _is_slicing{false};
};

}}}
