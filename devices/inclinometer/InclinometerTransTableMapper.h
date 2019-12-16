#pragma once

#include <vector>
#include <string>

#include <InclinometerTransTable.h>

namespace ros { namespace devices {

// ������ ����������� ������� ������������
struct InclinometerTransTableMapper {
public:
    static std::vector<InclinometerTransTableEntry> Load(const std::string& fileName);
};

}}
