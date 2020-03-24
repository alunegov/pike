#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <vector>

#include <Odometer.h>

namespace ros { namespace devices {

// �������/���������/������ ����������� ���������� (���-119)
class OdometerImpl : public Odometer
{
public:
    OdometerImpl() = delete;

    OdometerImpl(uint16_t a_channel, uint16_t b_channel, double_t threshold, double_t distance_per_pulse) :
        a_channel_{a_channel},
        b_channel_{b_channel},
        threshold_{threshold},
        distance_per_pulse_{distance_per_pulse}
    {}

    // Odometer

    void FillChannels(std::vector<uint16_t>& channels) override;

    void Update(const std::vector<uint16_t>& channels, const int16_t* values, size_t values_count,
            double_t adc_to_volt) override;

    double_t Get() override;

    void Reset() override;

private:
    std::array<std::vector<int16_t>, 2> ExtractChannelsValues(const std::vector<uint16_t>& channels,
            const int16_t* values, size_t values_count);

    uint64_t CalcPulses(const std::array<std::vector<int16_t>, 2>& channels_values, double_t adc_to_volt);

    // ����� ������ ��� ��� ������ A
    uint16_t a_channel_{0};
    // ����� ������ ��� ��� ������ B
    uint16_t b_channel_{0};

    // ������� �������������� ��������� 1 ��� 0 (����������� ����� ��� ����), �
    double_t threshold_{0};
    // �������� ����������� �� ���� �������, ��/�������
    double_t distance_per_pulse_{0};

    // ������� ��������� ������ A (0 ��� 1)
    bool a_channel_state_{false};
    // ����: ������� ��������� ������ A ����������������
    bool a_channel_state_inited_{false};
    // ���-�� �������, ������� � ������ ������
    std::atomic_int64_t pulses_{0};
};

}}
