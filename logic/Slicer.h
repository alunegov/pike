#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <functional>

namespace ros { namespace pike { namespace logic {

// Режим измерения сечения
class Slicer {
public:
    using CallbackFunc = void(double_t, int16_t);

    virtual ~Slicer() = default;

    virtual void Read(const std::atomic_bool& cancel_token, const std::function<CallbackFunc>& callback) = 0;
};

}}}
