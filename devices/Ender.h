#pragma once

#include <cstdint>

namespace ros { namespace devices {

// "Концевой" датчик
class Ender
{
public:
    virtual ~Ender() = default;

    // Обновляет состояние на основе ttl_in (полученное извне)
    virtual void Update(uint16_t ttl_in) = 0;

    // Возвращает последнее прочитанное состояние
    virtual bool Get() = 0;

    // Возвращает состояние - true, если на пин приходит 1, иначе false
    // Читает состояние с устройства.
    virtual bool Read() = 0;
};

}}
