#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <vector>

namespace ros { namespace devices {

struct TransTableEntry {
    double_t SinFi;
    double_t X;
    double_t Y;

    TransTableEntry(double_t sin_fi, double_t x, double_t y) :
        SinFi{sin_fi},
        X{x},
        Y{y}
    {}
};

struct ChannelTransTableEntry {
    double_t SinFi;
    double_t V;

    ChannelTransTableEntry(double_t sin_fi, double_t v) :
        SinFi{sin_fi},
        V{v}
    {}
};

// Инклинометр
// С датчика приходят два PWM-сигнала. Для упрощения считаем по ним СКЗ и используем таблицы пересчёта (вместо расчёта
// коэффициента заполнения).
// В классе повсеместно используется std::array<_Ty, 2>: первый элемент - канал X, второй элемент - канал Y.
class Inclinometer {
public:
    Inclinometer() = delete;

    // Ожидается, что значения TransTableEntry.X расположены по убыванию, а TransTableEntry.Y - по возрастанию.
    Inclinometer(uint16_t x_channel, uint16_t y_channel, std::vector<TransTableEntry> trans_table);

    void FillChannels(std::vector<uint16_t>& channels);

    void Update(const std::vector<uint16_t>& channels, const std::vector<int16_t>& values, double_t adc_to_volt);

    double_t Get();

private:
    // Расчёт значения по каналам регистрации (считаем СКЗ)
    std::array<double_t, 2> CalcChannelsValue(const std::vector<uint16_t>& channels,
            const std::vector<int16_t>& values, double_t adc_to_volt);

    // Преобразование значения по каналам регистрации в SinFi (по настроечным таблицам)
    std::array<double_t, 2> CalcChannelsFi(const std::array<double_t, 2>& channels_value);

    uint16_t x_channel_{0};
    uint16_t y_channel_{0};

    // Ожидается, что значения ChannelTransTableEntry.V расположены по убыванию.
    std::array<std::vector<ChannelTransTableEntry>, 2> channels_trans_table_;

    std::atomic<double_t> angle_{0};
};

}}
