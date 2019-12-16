#pragma once

namespace ros { namespace devices {

// "Концевой" датчик
class Ender {
public:
    virtual ~Ender() = default;

    virtual bool Read() = 0;
};

}}
