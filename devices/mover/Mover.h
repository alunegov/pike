#pragma once

#include <cstdint>

#include <LCardDevice.h>

namespace ros { namespace devices {

// ����������� ��������
enum class MoverDirection {
    Forward,   // �����
    Backward,  // �����
};

// ����������� �����-����� � ������� ������������� ���������
// ������� ��������� PWM 10���. �� ���������� ����� ������������ �������� (��������� �����. ����������). ����� ������
// ����������� ����� ����������� ������������� ���������.
class Mover {
public:
    Mover() = delete;

    Mover(ros::dc::lcard::LCardDevice* daq, uint16_t pwm_pin, uint16_t dir_pin) :
        daq_{daq},
        pwm_pin_{pwm_pin},
        dir_pin_{dir_pin}
    {}

    void SetDirection(MoverDirection direction);

    void Start();

    void Stop();

private:
    void applyDirection();

    ros::dc::lcard::LCardDevice* daq_{nullptr};
    uint16_t pwm_pin_{0};
    uint16_t dir_pin_{0};

    MoverDirection direction_{MoverDirection::Forward};
};

}}
