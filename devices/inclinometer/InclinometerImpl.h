#pragma once

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <vector>

#include <Inclinometer.h>

#include <InclinometerImplTransTable.h>

namespace ros { namespace devices {

// Строка в настроечной таблице инклинометра для преобразования значений канала (X или Y) в значение SinFi
// Это InclinometerImplTransTableEntry, разделённая на отдельные каналы для удобства обработки.
struct InclinometerImplChannelTransTableEntry
{
    // Значение SinFi
    double_t SinFi{0};
    // Значение канала, В
    double_t V{0};

    InclinometerImplChannelTransTableEntry(double_t sin_fi, double_t v) :
        SinFi{sin_fi},
        V{v}
    {}
};

// Инклинометр
// С датчика приходят два PWM-сигнала. Для упрощения считаем по ним СКЗ и используем таблицы пересчёта (вместо расчёта
// коэффициента заполнения).
// В классе повсеместно используется std::array<_Ty, 2>: первый элемент - канал X, второй элемент - канал Y.
class InclinometerImpl : public Inclinometer
{
public:
    using _Entry = InclinometerImplTransTableEntry;
    using _ChannelEntry = InclinometerImplChannelTransTableEntry;

    InclinometerImpl() = delete;

    // Ожидается, что значения InclinometerImplTransTableEntry.X расположены по убыванию, а
    // InclinometerImplTransTableEntry.Y - по возрастанию.
    InclinometerImpl(uint16_t x_channel, uint16_t y_channel, const std::vector<_Entry>& trans_table);

    // Inclinometer

    void FillChannels(std::vector<uint16_t>& channels) const override;

    void Update(const std::vector<uint16_t>& channels, const int16_t* values, size_t values_count,
            double_t adc_to_volt) override;

    double_t Get() const override;

private:
    // Расчёт значения по каналам регистрации (считаем СКЗ)
    std::array<double_t, 2> CalcChannelsValue(const std::vector<uint16_t>& channels, const int16_t* values,
            size_t values_count, double_t adc_to_volt) const;

    // Преобразование значения по каналам регистрации в SinFi (по настроечным таблицам)
    std::array<double_t, 2> CalcChannelsFi(const std::array<double_t, 2>& channels_value) const;

    static double_t CalcAngle(const std::array<double_t, 2>& channels_fi);

    uint16_t x_channel_{0};
    uint16_t y_channel_{0};

    // Ожидается, что значения InclinometerImplChannelTransTableEntry.V расположены по убыванию.
    std::array<std::vector<_ChannelEntry>, 2> channels_trans_table_;

    std::atomic<double_t> angle_{0};
};

}}
