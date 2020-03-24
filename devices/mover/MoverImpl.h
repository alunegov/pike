#pragma once

#include <cstdint>

#include <Mover.h>

#include <DAQ.h>

namespace ros { namespace devices {

// ����������� �����-����� � ������� ������������� ���������
// ������� ��������� PWM 10���. �� ���������� ����� ������������ �������� (��������� �����. ����������). ����� ������
// ����������� ����� ����������� ������������� ���������.
class MoverImpl : public Mover {
public:
    MoverImpl() = delete;

    MoverImpl(ros::dc::DAQ* daq, uint16_t pwm_pin, uint16_t dir_pin) :
        daq_{daq},
        pwm_pin_{pwm_pin},
        dir_pin_{dir_pin}
    {}

    ~MoverImpl() override;

    // Mover

    void SetDirection(MoverDirection direction);

    void Start();

    void Stop();

private:
    void applyDirection();

    ros::dc::DAQ* daq_{nullptr};
    uint16_t pwm_pin_{0};
    uint16_t dir_pin_{0};

    MoverDirection direction_{MoverDirection::Forward};
};

}}
