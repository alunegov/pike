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
struct AdcRateParams
{
    double_t FClock;
    uint32_t FClock_MinDiv;
    uint32_t FClock_MaxDiv;
    uint16_t IKD_MinKoeff;
    uint16_t IKD_MaxKoeff;
};

// Плата АЦП/ЦАП/ТТЛ от Л-Кард (через lcomp)
class LCardDaq : public DAQ
{
public:
    LCardDaq() = default;

    ~LCardDaq() override;

    // DAQ

    tl::expected<void, std::error_code> Init(size_t slot_num) override;

    tl::expected<void, std::error_code> Deinit() override;

    tl::expected<void, std::error_code> TtlEnable(bool enable) override;

    tl::expected<void, std::error_code> TtlOut(uint16_t value) override;

    tl::expected<void, std::error_code> TtlOut_SetPin(uint16_t value) override;

    tl::expected<void, std::error_code> TtlOut_ClrPin(uint16_t value) override;

    tl::expected<uint16_t, std::error_code> TtlIn() override;

    tl::expected<void, std::error_code> AdcRead(double_t& reg_freq, size_t point_count, const _Channels& channels,
            int16_t* values) override;

    tl::expected<void, std::error_code> AdcRead(double_t& reg_freq, const _Channels& channels,
            const std::atomic_bool& cancel_token, const std::function<AdcReadCallback>& callback) override;

private:
    tl::expected<void, std::error_code> NonVirtualDeinit();

    static const char* DetectBiosName(ULONG board_type);

    static AdcRateParams DetectAdcRateParams(ULONG board_type, const PLATA_DESCR_U2& plata_descr);

    ULONG PrepareAdc(double_t& reg_freq, const _Channels& channels, size_t* half_buffer, void** data, ULONG** sync);

    // копия функции ___GetRate из проекта UsbE_dll_v2
    static std::pair<uint32_t, uint16_t> GetRate(const AdcRateParams& rateParams, double_t channelRate,
            size_t channelCount, double_t eps);

    HINSTANCE lcomp_handle_{nullptr};
    IDaqLDevice* device_{nullptr};
    ULONG board_type_{NONE};
    AdcRateParams adc_rate_params_{};

    uint16_t ttl_out_value{0};
};

}}}
