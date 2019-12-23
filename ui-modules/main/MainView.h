#pragma once

#include <cmath>
#include <cstdint>
#include <functional>
#include <vector>

namespace ros { namespace pike { namespace modules {

// Вид главного окна
class MainView
{
public:
    virtual ~MainView() = default;

    virtual void RunOnUiThread(const std::function<void()>& f) = 0;

    virtual void SetDistance(double_t distance) = 0;

    virtual void SetAngle(double_t angle) = 0;

    virtual void SetDepth(int16_t depth) = 0;

    virtual void UpdateSliceDepth(double_t angle, int16_t depth) = 0;

    virtual void SetEnders(bool ender1, bool ender2) = 0;

    virtual void SetAdcChannels(const std::vector<uint16_t>& channels, const int16_t* values, size_t values_count,
            double_t adc_to_volt) = 0;

    virtual std::string GetDestPath() = 0;

    virtual void SetMoveForwardEnabled(bool enabled) = 0;

    virtual void SetMoveBackwardEnabled(bool enabled) = 0;

    virtual void SetRotateCcwEnabled(bool enabled) = 0;

    virtual void SetRotateCwEnabled(bool enabled) = 0;

    virtual void SetSliceEnabled(bool enabled) = 0;

    virtual void SliceCompleted() = 0;

    virtual void SetCamera1Enabled(bool enabled) = 0;

    virtual void SetCamera2Enabled(bool enabled) = 0;

    virtual void SetRecEnabled(bool enabled) = 0;

    virtual void SetPhotoEnabled(bool enabled) = 0;

    virtual void SetDestPathEnabled(bool enabled) = 0;
};

}}}
