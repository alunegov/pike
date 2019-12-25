#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <vector>

namespace ros { namespace pike { namespace logic {

struct SliceMsr
{
    bool ok{false};
    double_t inclio_angle{0};
    std::vector<double_t> angles;
    std::vector<int16_t> depths;
};

class SlicerReadOutput
{
public:
    virtual ~SlicerReadOutput() = default;

    virtual void SliceTick(double_t angle, int16_t depth) = 0;

    virtual void TtlInTick(bool ender1, bool ender2) = 0;
};

// Режим измерения сечения
class Slicer
{
public:
    virtual ~Slicer() = default;

    virtual SliceMsr Read(const std::atomic_bool& cancel_token, SlicerReadOutput* output) = 0;
};

}}}
