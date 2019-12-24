#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>

#include <Slicer.h>

#include <Pike.h>

namespace ros { namespace pike { namespace logic {

// Режим измерения сечения
class SlicerImpl : public Slicer
{
public:
    SlicerImpl() = delete;

    explicit SlicerImpl(ros::devices::Pike* pike) :
        pike_{pike}
    {}

    // Slicer

    SliceMsr Read(const std::atomic_bool& cancel_token, SlicerReadOutput* output) override;

private:
    ros::devices::Pike* pike_{nullptr};
};

}}}
