#include <InclinometerImpl.h>

#include <algorithm>
#include <cassert>
#include <cmath>

#include <RosMath.h>

namespace ros { namespace devices {

InclinometerImpl::InclinometerImpl(uint16_t x_channel, uint16_t y_channel, const std::vector<_Entry>& trans_table) :
    x_channel_{x_channel},
    y_channel_{y_channel}
{
    for (const _Entry& entry : trans_table) {
        channels_trans_table_[0].emplace_back(entry.SinFi, entry.X);
        channels_trans_table_[1].emplace_back(entry.SinFi, entry.Y);
    }

    // приводим Y к порядку X (по убыванию) - знание о порядке используется в CalcChannelsFi
    std::reverse(channels_trans_table_[1].begin(), channels_trans_table_[1].end());
}

void InclinometerImpl::FillChannels(_Channels& channels)
{
    channels.push_back(x_channel_);
    channels.push_back(y_channel_);
}

void InclinometerImpl::Update(const _Channels& channels, const _Values& values, double_t adc_to_volt)
{
    const auto channels_value = CalcChannelsValue(channels, values, adc_to_volt);

    const auto channels_fi = CalcChannelsFi(channels_value);

    const double_t sin_fi_x{channels_fi[0]};
    const double_t sin_fi_y{channels_fi[1]};

    double_t new_angle;

    if (sin_fi_x < (sqrt(2) / 2)) {
        if (sin_fi_y > 0) {
            new_angle = asin(sin_fi_x) * 180 / M_PI;
        } else {
            if (sin_fi_x > 0) {
                new_angle = 180 - asin(sin_fi_x) * 180 / M_PI;
            } else {
                // TODO: на блок-схеме этот кусок не влез
                new_angle = 0;
            }
        }
    } else {
        if (sin_fi_x > 0) {
            new_angle = 90 - asin(sin_fi_y) * 180 / M_PI;
        } else {
            new_angle = -90 + asin(sin_fi_y) * 180 / M_PI;
        }
    }

    angle_ = new_angle;
}

double_t InclinometerImpl::Get()
{
    return angle_;
}

std::array<double_t, 2> InclinometerImpl::CalcChannelsValue(const _Channels& channels, const _Values& values,
        double_t adc_to_volt)
{
    assert(!channels.empty());
    assert(values.size() >= channels.size());
    assert((values.size() % channels.size()) == 0);

    const size_t points_count{values.size() / channels.size()};

    std::array<uint16_t, 2> channels_num{x_channel_, y_channel_};
    std::array<double_t, 2> res{0, 0};

    std::transform(channels_num.begin(), channels_num.end(), res.begin(), [=](uint16_t channel_num) -> double_t {
        // поиск индекса канала в кадре по его номеру
        const auto channel = std::find(channels.begin(), channels.end(), channel_num);
        assert(channel != channels.end());
        const auto channel_index = std::distance(channels.begin(), channel);

        double_t rms = Rms_AdcRaw(values.data(), static_cast<uint32_t>(points_count),
                static_cast<uint32_t>(channels.size()), static_cast<uint32_t>(channel_index), 0);
        
        rms *= adc_to_volt;

        return rms;
    });

    return res;
}

std::array<double_t, 2> InclinometerImpl::CalcChannelsFi(const std::array<double_t, 2>& channels_value)
{
    const auto line = [](const _ChannelEntry& p1, const _ChannelEntry& p2, double_t x) -> double_t {
        return p1.SinFi + (x - p1.V) * (p2.SinFi - p1.SinFi) / (p2.V - p1.V);
    };

    std::array<double_t, 2> res{0, 0};

    std::transform(channels_value.begin(), channels_value.end(), channels_trans_table_.begin(), res.begin(),
        [=](double_t channel_value, const std::vector<_ChannelEntry>& channel_trans_table) -> double_t {
            assert(channel_trans_table.size() >= 2);

            if (channel_value >= channel_trans_table.front().V) {
                return channel_trans_table.front().SinFi;
            } else if (channel_value <= channel_trans_table.back().V) {
                return channel_trans_table.back().SinFi;
            } else {
                const auto entry = std::find_if(channel_trans_table.begin(), channel_trans_table.end(), [=](const _ChannelEntry& it) {
                    return channel_value <= it.V;
                });
                assert(entry != channel_trans_table.begin());
                assert(entry != channel_trans_table.end());
            
                return line(*(entry - 1), *entry, channel_value);
            }
        }
    );

    return res;
}

}}
