#include <MovementAndOrientationReader.h>

#include <cstdint>
#include <vector>

namespace ros { namespace pike { namespace logic {

void MovementAndOrientationReader::Start()
{
    double_t regFreq{12.8};
    std::vector<int16_t> values(1024 * 4);

    pike_->daq()->AdcRead(regFreq, 1024, {1 | 32, 2 | 32, 5 | 32, 6 | 32}, values.data());

    pike_->odometer()->Update(values);
    const auto distance = pike_->odometer()->Get();

    pike_->inclinometer()->Update(values);
    const auto angle = pike_->inclinometer()->Get();
}

}}}
