#pragma once

#include <cmath>
#include <cstdint>
#include <system_error>
#include <vector>

namespace ros { namespace pike { namespace logic {

// ������� ������������ �� ����� ������ OngoingReader
class OngoingReaderOutput
{
public:
    virtual ~OngoingReaderOutput() = default;

    // ����� �������� �� ��� (��������� ����������� ���������� � ��������� � ������������)
    virtual void AdcTick(double_t distance, double_t angle) = 0;

    // ����� �������� �� ��� (������� ������� �����������)
    virtual void AdcTick_Values(const std::vector<uint16_t>& channels, const int16_t* values, size_t values_count,
            double_t adc_to_volt) = 0;

    // ������ ��� (�� �����, ����� ������ ��� ��������)
    virtual void AdcError(const std::error_code& ec) = 0;

    // ����� �������� ������
    virtual void DepthTick(int16_t depth) = 0;

    // ����� �������� TtlIn (��������� ender)
    virtual void TtlInTick(bool ender1, bool ender2) = 0;
};

// ����� ��������� ����������� ����������, ��������� � ������������ � �������
// ��������� ��� ���������, ����� �������, ������� "������������������" �� ����� ��������� ������� (Slicer).
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
