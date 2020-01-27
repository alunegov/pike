#pragma once

#include <cmath>
#include <cstdint>
#include <system_error>
#include <vector>

namespace ros { namespace pike { namespace logic {

// События генерируемые во время работы OngoingReader
class OngoingReaderOutput
{
public:
    virtual ~OngoingReaderOutput() = default;

    // Новое значение от АЦП (показания пройденного расстояния и положения в пространстве)
    virtual void AdcTick(double_t distance, double_t angle) = 0;

    // Новое значение от АЦП (отсчёты каналов регистрации)
    virtual void AdcTick_Values(const std::vector<uint16_t>& channels, const int16_t* values, size_t values_count,
            double_t adc_to_volt) = 0;

    // Ошибка АЦП (по факту, поток чтения АЦП завершён)
    virtual void AdcError(const std::error_code& ec) = 0;

    // Новое значение губины
    virtual void DepthTick(int16_t depth) = 0;

    // Новое значение TtlIn (показания ender)
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

    virtual void Stop() = 0;

    virtual void IdleDepth(bool value) = 0;
};

}}}
