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
    assert(!channels.empty());
    assert(values.size() >= channels.size());
    assert((values.size() % channels.size()) == 0);

    // выделяем наши каналы из по-кадрового формата
    const size_t points_count{values.size() / channels.size()};
    
    std::array<uint16_t, 2> channels_num{a_channel_, b_channel_};
    std::array<_Values, 2> channels_values;

    std::transform(channels_num.begin(), channels_num.end(), channels_values.begin(), [=](const uint16_t channel_num) -> _Values {
        // поиск индекса канала в кадре по его номеру
        const auto channel = std::find(channels.begin(), channels.end(), channel_num);
        assert(channel != channels.end());
        const auto channel_index = std::distance(channels.begin(), channel);

        _Values res(points_count);

        ExtractChannelFromAdcFrames(values.data(), res.data(), points_count, channels.size(), channel_index);

        return res;
    });

    // считаем количество пульсов (передних фронтов) с учётом направления вращения
    const double_t threshold_adc{threshold_ / adc_to_volt};

    const auto is_pulse_front = [=](double_t it) { return it >= threshold_adc; };
    const auto is_pulse_back = [=](double_t it) { return it < threshold_adc; };

    const auto a_values = channels_values[0];
    const auto b_values = channels_values[1];

    int64_t a_pulses{0};

    auto tmp_begin = a_values.begin();
    if (!a_channel_state_inited_) {
        a_channel_state_inited_ = true;
        a_channel_state_ = is_pulse_front(*tmp_begin);
    }

    while (true) {
        if (a_channel_state_) {
            tmp_begin = std::find_if(tmp_begin, a_values.end(), is_pulse_back);
        } else {
            tmp_begin = std::find_if(tmp_begin, a_values.end(), is_pulse_front);
        }
        if (tmp_begin == a_values.end()) {
            break;
        }

        a_channel_state_ = is_pulse_front(*tmp_begin);

        if (a_channel_state_) {
            const auto i = std::distance(a_values.begin(), tmp_begin);

            // TODO: или наоборот?
            if (is_pulse_front(b_values[i])) {
                a_pulses++;
            } else {
                a_pulses--;
            }
        }
    }

    pulses_ += a_pulses;
}

double_t OdometerImpl::Get()
{
    return pulses_ * distance_per_pulse_;
}

void OdometerImpl::Reset()
{
    pulses_ = 0;
}

}}
