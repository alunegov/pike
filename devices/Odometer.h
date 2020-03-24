#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

namespace ros { namespace devices {

// �������/���������/������ ����������� ����������
class Odometer {
public:
    using _Channels = std::vector<uint16_t>;
    using _Values = std::vector<int16_t>;

    virtual ~Odometer() = default;

    // ��������� ������ ������� ��� ����������� � ���
    virtual void FillChannels(_Channels& channels) = 0;

    // ��������� ���������� ���������� �� ������������������� ������� � �������
    virtual void Update(const _Channels& channels, const _Values& values, double_t adc_to_volt) = 0;

    // ���������� ���������� ����������, ��
    // ����������� ������� ������������ Conf::odometer::distance_per_pulse.
    virtual double_t Get() = 0;

    // �������� ���������� ����������
    virtual void Reset() = 0;
};

}}
