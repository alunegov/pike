#pragma once

#include <cmath>

namespace ros { namespace devices {

// ����������� ��������
enum class RotatorDirection
{
    CW,   // �� ������� ������� (�� ����������� ��������)
    CCW,  // ������ ������� ������� (�� ����������� ��������)
};

// �������� ��������
enum class RotatorSpeed
{
    Low,   // 1/32 step
    High,  // 1/2 step
};

// �������� �������������� �����
class Rotator
{
public:
    virtual ~Rotator() = default;

    // ����� ����������� ��������
    // ���������� ������������ ������ � Start ��� Rotate.
    virtual void SetDirection(RotatorDirection direction) = 0;

    // ����� �������� ��������
    // ���������� ������������ ������ � Start ��� Rotate.
    virtual void SetSpeed(RotatorSpeed speed) = 0;

    // ���������� ���������� ����� ��� �������� �� 360�
    // ���������� ���������� ��� ��������, � �� ������������ �������� - ��� �������� ����� ���� ��������, ���� �����
    // ��� �������� ������������ Step.
    virtual uint32_t StepsIn360() = 0;

    // ��������� ��������
    // ����� ������� ���������� ����������� � ��������.
    // ��� Rotate, ������ ���������� � ���� �� ���������.
    virtual void Start() = 0;

    // ������������� ��������
    virtual void Stop() = 0;

    // ������������ �� ��������� ����� �����
    // ����� ������� ���������� ����������� � ��������.
    // ��� Start, ������ ��������� � �� ��������� ���-�� �����.
    virtual void Rotate(size_t steps_count = 1) = 0;

    // ��������� �������� (low-level)
    virtual void Enable() = 0;

    // ��������� �������� (low-level)
    virtual void Disable() = 0;

    // ������������ �� ���� ��� (low-level)
    virtual void Step() = 0;
};

}}
