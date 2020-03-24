#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

namespace ros { namespace devices {

// �������/���������/������ ����������� ����������
class Odometer {
public:
    virtual ~Odometer() = default;

    // ��������� ������ ������� ��� ����������� � ���
    virtual void FillChannels(std::vector<uint16_t>& channels) = 0;

    // ��������� ���������� ���������� �� ������������������� ������� � �������
    virtual void Update(const std::vector<uint16_t>& channels, const int16_t* values, size_t values_count,
            double_t adc_to_volt) = 0;

    // ���������� ���������� ����������, ��
    // ����������� ������� ������������ Conf::odometer::distance_per_pulse.
    virtual double_t Get() = 0;

    // �������� ���������� ����������
    virtual void Reset() = 0;
};

}}
