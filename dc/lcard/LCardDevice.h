#pragma once

#include <cmath>
#include <cstdint>
#include <vector>
#include <utility>

#define INITGUID
#include <Windows.h>

#include <ioctl.h>
#include <ifc_ldev.h>

#include <DAQ.h>

namespace ros { namespace dc { namespace lcard {

// Параметры платы Л-Кард, используемые для настройки частоты АЦП (межкадровой и межканальной задержек)
struct AdcRateParams {
    double_t FClock;
    uint32_t FClock_MinDiv;
    uint32_t FClock_MaxDiv;
    uint16_t IKD_MinKoeff;
    uint16_t IKD_MaxKoeff;
};

// Плата АЦП/ЦАП/ТТЛ от Л-Кард (через lcomp)
class LCardDevice : public DAQ {
public:
    LCardDevice() = default;

    ~LCardDevice() override;

    // DAQ

    void Init(size_t slot_num) override;

    void Deinit() override;

    void TtlEnable(bool enable) override;

    void TtlOut(uint16_t value) override;

    void TtlOut_SetPin(uint16_t value) override;

    void TtlOut_ClrPin(uint16_t value) override;

    uint16_t TtlIn() override;

    void AdcRead(double_t& reg_freq, size_t point_count, const std::vector<uint16_t>& channels, int16_t* values) override;

private:
    static const char* DetectBiosName(ULONG board_type);

    static AdcRateParams DetectAdcRateParams(ULONG board_type, const PLATA_DESCR_U2& plata_descr);

    // копия функции ___GetRate из проекта UsbE_dll_v2
    std::pair<uint32_t, uint16_t> GetRate(const AdcRateParams& rateParams, double_t channelRate, size_t channelCount,
            double_t eps);

    HINSTANCE lcomp_handle_{0};
    IDaqLDevice* device_{nullptr};
    ULONG board_type_{NONE};
    AdcRateParams adc_rate_params_{};

    uint16_t ttl_out_value{0};
};

}}}
