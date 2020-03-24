#pragma once

#include <cmath>

namespace ros { namespace devices {

// ������ � ����������� ������� ������������ ��� �������������� �������� ������� (X � Y) � �������� SinFi
struct InclinometerImplTransTableEntry
{
    // �������� SinFi
    double_t SinFi{0};
    // �������� ������ X, �
    double_t X{0};
    // �������� ������ Y, �
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
