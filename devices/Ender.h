#pragma once

#include <cstdint>

namespace ros { namespace devices {

// "��������" ������
class Ender
{
public:
    virtual ~Ender() = default;

    // ��������� ��������� �� ������ ttl_in (���������� �����)
    virtual void Update(uint16_t ttl_in) = 0;

    // ���������� ��������� ����������� ���������
    virtual bool Get() = 0;

    // ���������� ��������� - true, ���� �� ��� �������� 1, ����� false
    // ������ ��������� � ����������.
    virtual bool Read() = 0;
};

}}
