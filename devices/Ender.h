#pragma once

namespace ros { namespace devices {

// "��������" ������
class Ender {
public:
    virtual ~Ender() = default;

    virtual bool Read() = 0;
};

}}
