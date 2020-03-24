#pragma once

#include <cmath>
#include <system_error>

#include <tl/expected.hpp>

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

class RotatorOutput
{
public:
    virtual ~RotatorOutput() = default;

    virtual void RotateError(const std::error_code& ec) = 0;
};

// �������� �������������� �����
class Rotator
{
public:
    virtual ~Rotator() = default;

    virtual void SetOutput(RotatorOutput* output) = 0;

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
    virtual tl::expected<void, std::error_code> Start() = 0;

    // ������������� ��������
    virtual void Stop() = 0;

    // ������������ �� ��������� ����� �����
    // ����� ������� ���������� ����������� � ��������.
    // ��� Start, ������ ��������� � �� ��������� ���-�� �����.
    virtual tl::expected<void, std::error_code> Rotate(size_t steps_count = 1) = 0;

    // ��������� �������� (low-level)
    virtual tl::expected<void, std::error_code> Enable() = 0;

    // ��������� �������� (low-level)
    virtual tl::expected<void, std::error_code> Disable() = 0;

    // ������������ �� ���� ��� (low-level)
    virtual tl::expected<void, std::error_code> Step() = 0;
};

}}
