#pragma once

#include <cmath>

namespace ros { namespace devices {

// Строка в настроечной таблице инклинометра для преобразования значений каналов (X и Y) в значение SinFi
struct InclinometerImplTransTableEntry
{
    // Значение SinFi
    double_t SinFi{0};
    // Значение канала X, В
    double_t X{0};
    // Значение канала Y, В
    double_t Y{0};

    InclinometerImplTransTableEntry(double_t sin_fi, double_t x, double_t y) :
        SinFi{sin_fi},
        X{x},
        Y{y}
    {}

    bool operator ==(const InclinometerImplTransTableEntry& ref) const
    {
        return (SinFi == ref.SinFi) && (X == ref.X) && (Y == ref.Y);
    }
};

}}
