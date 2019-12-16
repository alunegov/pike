#include <OdometerImpl.h>

#include <array>
#include <cassert>
#include <vector>

#include <RosMath.h>

namespace ros { namespace devices {

void OdometerImpl::FillChannels(_Channels& channels)
{
    channels.push_back(a_channel_);
    channels.push_back(b_channel_);
}

void OdometerImpl::Update(const _Channels& channels, const _Values& values, double_t adc_to_volt)
{
    const size_t points_count{values.size() / channels.size()};
    const double_t threshold_adc{threshold_ / adc_to_volt};
    
    std::array<uint16_t, 2> channels_num{a_channel_, b_channel_};
    std::array<_Values, 2> channels_values;

    std::transform(channels_num.begin(), channels_num.end(), channels_values.begin(), [=](const uint16_t channel_num) -> _Values {
        const auto channel = std::find(channels.begin(), channels.end(), channel_num);
        assert(channel != channels.end());

        const auto channel_index = std::distance(channels.begin(), channel);

        _Values res(points_count);

        ExtractChannelFromAdcFrames(values.data(), res.data(), points_count, channels.size(), channel_index);

        return res;
    });

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
