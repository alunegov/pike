#pragma once

namespace ros { namespace devices {

// "��������" ������
class Ender
{
public:
    virtual ~Ender() = default;

    // ���������� true, ���� �� ��� �������� 1, ����� false
    virtual bool Read() = 0;
};

}}
