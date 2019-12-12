#include <Odometer.h>

namespace ros { namespace devices {

void Odometer::Update(const std::vector<int16_t>& values)
{
    int32_t movement{0};

    distance_ += movement;
}

int32_t Odometer::Get()
{
    return distance_;
}

void Odometer::Reset()
{
    distance_ = 0;
}

}}
