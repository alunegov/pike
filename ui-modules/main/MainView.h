#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

namespace ros { namespace pike { namespace modules {

// Вид главного окна
class MainView
{
public:
    virtual ~MainView() = default;

    virtual void SetDistance(double_t value) = 0;

    virtual void SetAngle(double_t value) = 0;

    virtual void SetDepth(int16_t value) = 0;

    virtual void UpdateSliceDepth(double_t angle, int16_t depth) = 0;

    virtual void SetEnders(bool ender1, bool ender2) = 0;

    virtual void SetAdcChannels(const std::vector<uint16_t>& channels, const int16_t* values, size_t values_count,
            double_t adc_to_volt) = 0;
};

}}}
