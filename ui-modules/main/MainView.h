#pragma once

#include <cstdint>

namespace ros { namespace pike { namespace modules {

// Вид главного окна
class MainView {
public:
    virtual ~MainView() = default;

    virtual void SetDistance(int32_t value) = 0;

    virtual void SetAngle(double_t value) = 0;

    virtual void SetDepth(int16_t value) = 0;

    virtual void UpdateSliceDepth(double_t angle, int16_t depth) = 0;
};

}}}
