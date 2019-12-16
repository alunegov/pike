#pragma once

#include <vector>
#include <string>

#include <InclinometerImplTransTable.h>

namespace ros { namespace devices {

// Маппер настроечной таблицы инклинометра
struct InclinometerImplTransTableMapper {
public:
    static std::vector<InclinometerImplTransTableEntry> Load(const std::string& fileName);
};

}}
