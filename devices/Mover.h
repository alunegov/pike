#pragma once

namespace ros { namespace devices {

// ����������� ��������
enum class MoverDirection {
    Forward,   // �����
    Backward,  // �����
};

// ����������� �����-�����
class Mover {
public:
    virtual ~Mover() = default;

    virtual void SetDirection(MoverDirection direction) = 0;

    virtual void Start() = 0;

    virtual void Stop() = 0;
};

}}
