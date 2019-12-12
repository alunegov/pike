#pragma once

#include <cmath>
#include <cstdint>
#include <vector>
#include <utility>

#define INITGUID
#include <Windows.h>

#include <ioctl.h>
#include <ifc_ldev.h>

namespace ros { namespace dc { namespace lcard {

struct AdcRateParams {
    double_t FClock;
    uint32_t FClock_MinDiv;
    uint32_t FClock_MaxDiv;
    uint16_t IKD_MinKoeff;
    uint16_t IKD_MaxKoeff;
};

class LCardDevice {
public:
    LCardDevice() = default;

    virtual ~LCardDevice();

    void Init(ULONG slot_num);

    void Deinit();

    void TtlEnable(bool enable);

    void TtlOut(uint16_t value);

    void TtlOut_SetPin(uint16_t value);

    void TtlOut_ClrPin(uint16_t value);

    uint16_t TtlIn();

    void AdcRead(double_t& reg_freq, size_t point_count, const std::vector<uint16_t>& channels, int16_t* values);

private:
    static const char* DetectBiosName(ULONG board_type);

    static AdcRateParams DetectAdcRateParams(ULONG board_type, const PLATA_DESCR_U2& plata_descr);

    // копия функции ___GetRate из проекта UsbE_dll_v2
    std::pair<uint32_t, uint16_t> GetRate(const AdcRateParams& rateParams, double_t channelRate, size_t channelCount,
            double_t eps);

    HINSTANCE lcomp_handle_{0};
    IDaqLDevice* device_{nullptr};
    ULONG board_type_{NONE};
    AdcRateParams adc_rate_params_;

    uint16_t ttl_out_value{0};
};

}}}
