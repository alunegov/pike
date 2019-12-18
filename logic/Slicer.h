#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <functional>

namespace ros { namespace pike { namespace logic {

class SlicerReadOutput {
public:
    virtual void SliceTick(double_t angle, int16_t depth) = 0;

    virtual void TtlInTick(bool ender1, bool ender2) = 0;
};

// Режим измерения сечения
class Slicer {
public:
    virtual ~Slicer() = default;

    virtual void Read(const std::atomic_bool& cancel_token, SlicerReadOutput* output) = 0;
};

}}}
