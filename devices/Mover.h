#pragma once

#include <system_error>

#include <tl/expected.hpp>

namespace ros { namespace devices {

// ����������� ��������
enum class MoverDirection {
    Forward,   // �����
    Backward,  // �����
};

// ����������� �����-�����
class Mover
{
public:
    virtual ~Mover() = default;

    // ����� ����������� �����������
    // ���������� ������������ ������ � Start.
    virtual void SetDirection(MoverDirection direction) = 0;

    // ��������� �����������
    // ����� ������� ������������� ��������� ����������� � ���������� �����������.
    virtual tl::expected<void, std::error_code> Start() = 0;

    // ������������� �����������
    virtual tl::expected<void, std::error_code> Stop() = 0;
};

}}
