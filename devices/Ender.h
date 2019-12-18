#pragma once

namespace ros { namespace devices {

// "Концевой" датчик
class Ender
{
public:
    virtual ~Ender() = default;

    // Возвращает true, если на пин приходит 1, иначе false
    virtual bool Read() = 0;
};

}}
