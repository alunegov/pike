#pragma once

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <vector>

#include <InclinometerTransTable.h>

namespace ros { namespace devices {

// Строка в настроечной таблице инклинометра для преобразования значений канала (X или Y) в значение SinFi
// Это InclinometerTransTableEntry, разделённая на отдельные каналы для удобства обработки.
struct InclinometerChannelTransTableEntry {
    // Значение SinFi
    double_t SinFi;
    // Значение канала, В
    double_t V;

    InclinometerChannelTransTableEntry(double_t sin_fi, double_t v) :
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

    // Ожидается, что значения InclinometerTransTableEntry.X расположены по убыванию, а InclinometerTransTableEntry.Y
    // - по возрастанию.
    Inclinometer(uint16_t x_channel, uint16_t y_channel, const std::vector<InclinometerTransTableEntry>& trans_table);

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

    // Ожидается, что значения InclinometerChannelTransTableEntry.V расположены по убыванию.
    std::array<std::vector<InclinometerChannelTransTableEntry>, 2> channels_trans_table_;

    std::atomic<double_t> angle_{0};
};

}}
