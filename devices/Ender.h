#pragma once

#include <cstdint>
#include <system_error>

#include <tl/expected.hpp>

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
    virtual tl::expected<bool, std::error_code> Read() = 0;
};

}}
