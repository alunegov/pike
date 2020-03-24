#pragma once

#include <cmath>
#include <cstdint>
#include <functional>

namespace ros { namespace pike { namespace logic {

// ����� ��������� ����������� ����������, ��������� � ������������ � �������
// ��������� ��� ���������, ����� �������, ������� "������������������" �� ����� ��������� ������� (Slicer).
class OngoingReader {
public:
    using CallbackFunc = void(int32_t, double_t, int16_t);

    virtual ~OngoingReader() = default;

    virtual void Start(const std::function<CallbackFunc>& callback) = 0;

    virtual void IdleDepth(bool value) = 0;
};

}}}
