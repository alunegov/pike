#pragma once

#include <cmath>

namespace ros { namespace devices {

// ������ � ����������� ������� ������������ ��� �������������� �������� ������� (X � Y) � �������� SinFi
struct InclinometerTransTableEntry {
    // �������� SinFi
    double_t SinFi;
    // �������� ������ X, �
    double_t X;
    // �������� ������ Y, �
    double_t Y;

    InclinometerTransTableEntry(double_t sin_fi, double_t x, double_t y) :
        SinFi{sin_fi},
        X{x},
        Y{y}
    {}
};

}}
