#pragma once

#include <vector>
#include <string>

#include <InclinometerImplTransTable.h>

namespace ros { namespace devices {

// Маппер настроечной таблицы инклинометра
struct InclinometerImplTransTableMapper
{
public:
    // Загружает настроечную таблицу из tbl-файла
    static std::vector<InclinometerImplTransTableEntry> Load(const std::string& fileName);
};

}}
