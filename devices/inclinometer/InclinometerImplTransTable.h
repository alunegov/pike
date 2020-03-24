#pragma once

#include <cmath>

namespace ros { namespace devices {

// ������ � ����������� ������� ������������ ��� �������������� �������� ������� (X � Y) � �������� SinFi
struct InclinometerImplTransTableEntry {
    // �������� SinFi
    double_t SinFi;
    // �������� ������ X, �
    double_t X;
    // �������� ������ Y, �
    double_t Y;

    InclinometerImplTransTableEntry(double_t sin_fi, double_t x, double_t y) :
        SinFi{sin_fi},
        X{x},
        Y{y}
    {}
};

}}
