#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <system_error>

#include <tl/expected.hpp>

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

    SliceMsr Read(const std::atomic_bool& cancel_token, SlicerReadOutput* output) const override;

private:
    // поворот в крайнее левое положение
    tl::expected<void, std::error_code> rotate_to_start(const std::atomic_bool& cancel_token, SlicerReadOutput* output) const;

    // измерение, поворачивая в крайнее правое положение
    tl::expected<void, std::error_code> read_slice(const std::atomic_bool& cancel_token, SlicerReadOutput* output,
            SliceMsr& slice_msr) const;

    ros::pike::logic::Pike* pike_{nullptr};
};

}}}
