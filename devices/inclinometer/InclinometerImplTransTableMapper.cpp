#include <InclinometerImplTransTableMapper.h>

#include <fstream>

namespace ros { namespace devices {

std::vector<InclinometerImplTransTableEntry> InclinometerImplTransTableMapper::Load(const std::string& fileName)
{
    std::vector<InclinometerImplTransTableEntry> res;

    std::ifstream file{fileName};

    double_t sin_fi, x, y;
    while (file >> sin_fi >> x >> y) {
        res.emplace_back(sin_fi, x, y);
    }

    return res;
}

}}
