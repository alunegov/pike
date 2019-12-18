#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

namespace ros { namespace pike { namespace logic {

class OngoingReaderOutput
{
public:
    virtual void AdcTick(double_t distance, double_t angle, int16_t depth) = 0;

    virtual void AdcTick_Values(const std::vector<uint16_t>& channels, const int16_t* values, size_t values_count,
            double_t adc_to_volt) = 0;

    virtual void TtlInTick(bool ender1, bool ender2) = 0;
};

// Режим измерения пройденного расстояния, положения в пространстве и глубины
// Измерение идёт постоянно, кроме глубины, которая "приостанавливается" на время измерения сечения (Slicer).
class OngoingReader
{
public:
    virtual ~OngoingReader() = default;

    virtual void SetOutput(OngoingReaderOutput* output) = 0;

    virtual void Start() = 0;

    virtual void IdleDepth(bool value) = 0;
};

}}}
