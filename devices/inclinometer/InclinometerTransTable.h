#pragma once

#include <cmath>

namespace ros { namespace devices {

// Строка в настроечной таблице инклинометра для преобразования значений каналов (X и Y) в значение SinFi
struct InclinometerTransTableEntry {
    // Значение SinFi
    double_t SinFi;
    // Значение канала X, В
    double_t X;
    // Значение канала Y, В
    double_t Y;

    InclinometerTransTableEntry(double_t sin_fi, double_t x, double_t y) :
        SinFi{sin_fi},
        X{x},
        Y{y}
    {}
};

}}
