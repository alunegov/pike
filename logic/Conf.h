#pragma once

#include <cmath>
#include <cstdint>
#include <string>

namespace ros { namespace pike { namespace logic {

// ���������
// ��������� ������� ���/��� � 1.
struct Conf
{
    // ����� ������� ���������, ��
    double_t object_length;

    // ��������� ����� ����� ������ �-����
    struct
    {
        // ����� �����, ��������� � 0
        size_t slot;
        // ����: ����� ����� � ������� ��� (����� ����. �����������)
        bool common_gnd;
        // ������� ����������� ���, ���
        double_t adc_rate;
    } daq;

    // ��������� ������� ������� CD22
    struct
    {
        // ��� COM-�����, � ������� COM1 ��� \\.\COM43
        std::string port_name;
        // �������� �����, ���
        int32_t baud_rate;
    } depthometer;

    // ��������� �1
    struct
    {
        // ����� ������� ����� ���
        uint16_t pin;
    } ender1;

    // ��������� �2
    struct
    {
        // ����� ������� ����� ���
        uint16_t pin;
    } ender2;

    // ��������� ������������
    struct
    {
        // ����� ������ ��� ��� ������ X
        uint16_t x_channel;
        // ����� ������ ��� ��� ������ Y
        uint16_t y_channel;
        // ��� ����� � �������� ��������� � �������
        std::string trans_table_file;
    } inclinometer;

    // ��������� ������������� ���������
    struct
    {
        // ����� �������� ����� ��� ��� PWM
        uint16_t pwm_pin;
        // ����� �������� ����� ��� ��� Dir
        uint16_t dir_pin;
    } mover;

    // ��������� ������� ���������� ���
    struct
    {
        // ����� ������ ��� ��� ������ A
        uint16_t a_channel;
        // ����� ������ ��� ��� ������ B
        uint16_t b_channel;
        // ������� ��� �������������� "�������", �
        double_t threshold;
        // ��������� �� ���� "�����", ��
        double_t distance_per_pulse;
    } odometer;

    // ��������� �������� ���������
    struct
    {
        // ����� �s������ ����� TTL ��� En
        uint16_t en_pin;
        // ����� �s������ ����� TTL ��� Step
        uint16_t step_pin;
        // ����� �s������ ����� TTL ��� Dir
        uint16_t dir_pin;
        // ����� �s������ ����� TTL ��� M0
        uint16_t mx_pin;
        // ���������� ����� � ������ ������� � ������ ��������� (M0 = 0)
        uint32_t steps_per_msr;
        // ���������� ����� � ������ ������� � ������ ������� �������� (M0 = 1)
        uint32_t steps_per_view;
    } rotator;

    // ��������� ������� "���������" ���������� ���������
    struct
    {
        // ����� UDP-����� ��� ����� ��������� �� "���������" ���������
        uint16_t port;
    } remote;
};

}}}
