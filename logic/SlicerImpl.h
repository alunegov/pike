#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>

#include <Pike.h>
#include <Slicer.h>

namespace ros { namespace pike { namespace logic {

// Режим измерения сечения
class SlicerImpl : public Slicer
{
public:
    SlicerImpl() = delete;

    explicit SlicerImpl(ros::pike::logic::Pike* pike) :
        pike_{pike}
    {}

    // Slicer

    SliceMsr Read(const std::atomic_bool& cancel_token, SlicerReadOutput* output) override;

private:
    ros::pike::logic::Pike* pike_{nullptr};
};

}}}
