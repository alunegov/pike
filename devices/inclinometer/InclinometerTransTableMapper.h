#pragma once

#include <vector>
#include <string>

#include <InclinometerTransTable.h>

namespace ros { namespace devices {

// Маппер настроечной таблицы инклинометра
struct InclinometerTransTableMapper {
public:
    static std::vector<InclinometerTransTableEntry> Load(const std::string& fileName);
};

}}
