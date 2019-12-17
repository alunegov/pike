#pragma once

#include <cmath>
#include <cstdint>
#include <functional>

namespace ros { namespace pike { namespace logic {

// Режим измерения пройденного расстояния, положения в пространстве и глубины
// Измерение идёт постоянно, кроме глубины, которая "приостанавливается" на время измерения сечения (Slicer).
class OngoingReader {
public:
    using CallbackFunc = void(double_t distance, double_t angle, int16_t depth);

    virtual ~OngoingReader() = default;

    virtual void Start(const std::function<CallbackFunc>& callback) = 0;

    virtual void IdleDepth(bool value) = 0;
};

}}}
