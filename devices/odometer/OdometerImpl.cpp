#include <OdometerImpl.h>

namespace ros { namespace devices {

void OdometerImpl::FillChannels(std::vector<uint16_t>& channels)
{
    channels.push_back(a_channel_);
    channels.push_back(b_channel_);
}

void OdometerImpl::Update(const std::vector<uint16_t>& channels, const std::vector<int16_t>& values, double_t adc_to_volt)
{
    int32_t movement{0};

    distance_ += movement;
}

int32_t OdometerImpl::Get()
{
    return distance_;
}

void OdometerImpl::Reset()
{
    distance_ = 0;
}

}}