#pragma once

#include <string>

#include <Conf.h>

namespace ros { namespace pike { namespace logic {

// Маппер настроек
class ConfMapper {
public:
    // Загружает настройки из json-файла
    static Conf Load(const std::string& filename);
};

}}}
