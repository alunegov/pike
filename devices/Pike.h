#pragma once

#include <cstdint>

#include <DAQ.h>
#include <Depthometer.h>
#include <Ender.h>
#include <Inclinometer.h>
#include <Mover.h>
#include <Odometer.h>
#include <Rotator.h>

namespace ros { namespace devices {

// "����"
class Pike
{
public:
    virtual ~Pike() = default;

    // ���������� ����� ���/���/���
    virtual ros::dc::DAQ* daq() const = 0;

    // ���������� "��������" ������ 1
    virtual ros::devices::Ender* ender1() const = 0;

    // ���������� "��������" ������ 2
    virtual ros::devices::Ender* ender2() const = 0;

    // ���������� ��������� �������������� ����� � ������������� � ��������
    virtual ros::devices::Rotator* rotator() const = 0;

    // ���������� ������������
    virtual ros::devices::Mover* mover() const = 0;

    // ���������� �������
    virtual ros::devices::Odometer* odometer() const = 0;

    // ���������� �����������
    virtual ros::devices::Inclinometer* inclinometer() const = 0;

    // ���������� �����������
    virtual ros::devices::Depthometer* depthometer() const = 0;
};

}}
