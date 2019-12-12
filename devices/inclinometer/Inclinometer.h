#pragma once

#include <atomic>
#include <cstdint>
#include <vector>

namespace ros { namespace devices {

// �����������
// � ������� �������� ��� PWM-�������. ��� ��������� ������� �� ��� ��� � ���������� ������� ��������� (������ �������
// ������������ ����������).
class Inclinometer {
public:
    Inclinometer() = delete;

    Inclinometer(uint16_t x_channel, uint16_t y_channel) :
        x_channel_{x_channel},
        y_channel_{y_channel}
    {}

    void Update(const std::vector<int16_t>& values);

    double_t Get();

private:
    uint16_t x_channel_{0};
    uint16_t y_channel_{0};

    std::atomic<double_t> angle_{0};
};

}}
