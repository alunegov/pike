#include <DchwBridge.h>

#include <algorithm>
#include <cassert>
#include <memory>
#include <thread>

#include <Windows.h>

#include <LCardDaq.h>

struct ModulAdapter
{
    ros::dc::lcard::LCardDaq daq;

    std::atomic_bool cancel_token;

    std::thread adc_read_thread;
};

DC_LCARD_API
uint16_t __stdcall GetDllProtoVer(void)
{
    return DllProtoVer;
}

/*DC_LCARD_API
void __stdcall SetLang(const char* lang)
{
    (void)lang;

    // nop
}*/

DC_LCARD_API
uint8_t __stdcall InitDevice(uint8_t ModulType, uint8_t NumberSlot, struct UsbLModul** UsbLModul)
{
    assert(UsbLModul);
    assert(!(*UsbLModul));

    auto modul_adapter = std::make_unique<ModulAdapter>();
    ros::dc::lcard::LCardDaq& daq{modul_adapter->daq};

    const auto init_opt = daq.Init(NumberSlot);
    if (!init_opt) {
        assert(init_opt.error().value() <= UINT8_MAX);
        return static_cast<uint8_t>(init_opt.error().value());
    }

    // ModulType (см. DcDeviceExt_Kamerton.TDchwModuleType, код плюс 1) должен совпадать с типом загруженного модуля
    switch (ModulType) {
    case 1:
        if (daq.GetBoardType() != E440) {
            return 1;
        }
        break;
    case 2:
        if (daq.GetBoardType() != E140) {
            return 1;
        }
        break;
    case 3:
        if ((daq.GetBoardType() != E2010) && (daq.GetBoardType() != E2010B)) {
            return 1;
        }
        break;
    case 4:
        if (daq.GetBoardType() != E154) {
            return 1;
        }
        break;
    default:
        return 1;
    }

    auto modul = std::make_unique<struct UsbLModul>();

    modul->ModuleType = ModulType;
    //modul->ModulState не используем
    modul->Modul = modul_adapter.release();
    modul->UsbSpeed = 1;  // Предполагаем, что USB 2.0. В IDaqLDevice нет соответсвующего метода
    //modul->RMD не используем
    modul->Revision = daq.GetRevision();  // daq ещё валиден, хотя modul_adapter уже нет

    *UsbLModul = modul.release();

    return 0;
}

DC_LCARD_API
uint8_t __stdcall DestroyDevice(struct UsbLModul** UsbLModul)
{
    assert(UsbLModul);

    if (!(*UsbLModul)) {
        return 0;
    }

    auto const modul_adapter = static_cast<ModulAdapter*>((*UsbLModul)->Modul);
    assert(modul_adapter);
    
    modul_adapter->cancel_token = true;
    if (modul_adapter->adc_read_thread.joinable()) {
        modul_adapter->adc_read_thread.join();
    }

    /*const auto deinit_opt = */modul_adapter->daq.Deinit();

    delete (*UsbLModul)->Modul;
    delete *UsbLModul;
    *UsbLModul = nullptr;

    return 0;
}

DC_LCARD_API
const char* __stdcall GetErrorText2(uint8_t Number)
{
    (void)Number;

    // TODO: 
    return "error";
}

DC_LCARD_API
struct Point __stdcall GetRate(const struct UsbLModul* aModule, double_t aChannelRate, uint8_t* aChannelCount)
{
    (void)aModule;
    (void)aChannelRate;
    (void)aChannelCount;

    return {0, 0};
}

DC_LCARD_API
uint8_t __stdcall Break_Adc(struct UsbLModul* pULM, struct OptionsRead* pOR)
{
    assert(pULM);
    assert(pOR);

    auto const modul_adapter = static_cast<ModulAdapter*>(pULM->Modul);
    assert(modul_adapter);

    modul_adapter->cancel_token = true;
    if (modul_adapter->adc_read_thread.joinable()) {
        modul_adapter->adc_read_thread.join();
    }

    if (pOR->BreakEvent) {
        SetEvent(pOR->BreakEvent);
    }

    return 0;
}

DC_LCARD_API
uint8_t __stdcall Slow_Adc(struct UsbLModul* pULM, struct ParamRead* AdcParam, struct OptionsRead* pOR)
{
    assert(pULM);
    assert(AdcParam);
    assert(pOR);

    auto const modul_adapter = static_cast<ModulAdapter*>(pULM->Modul);
    assert(modul_adapter);

    modul_adapter->cancel_token = false;

    modul_adapter->adc_read_thread = std::thread{[modul_adapter, AdcParam, pOR]() {
        AdcParam->State = 0;

        std::vector<uint16_t> channels(AdcParam->NumbCh);
        std::copy(std::begin(AdcParam->ChTab), std::begin(AdcParam->ChTab) + AdcParam->NumbCh, std::begin(channels));

        const auto callback = [pOR](size_t cur_values_count) {
            assert(cur_values_count <= UINT32_MAX);
            pOR->cbBuffCount += static_cast<uint32_t>(cur_values_count);
            if (pOR->cbBuffFilledEvent) {
                SetEvent(pOR->cbBuffFilledEvent);
            }
        };

        const auto adc_read_opt = modul_adapter->daq.AdcRead(AdcParam->RealChRate, AdcParam->NumbChStep, channels,
                static_cast<int16_t*>(AdcParam->pBuffer), callback, modul_adapter->cancel_token);

        if (!modul_adapter->cancel_token) {
            if (adc_read_opt) {
                if (pOR->AdcFinishRead) {
                    SetEvent(pOR->AdcFinishRead);
                }
            } else {
                assert(adc_read_opt.error().value() <= UINT8_MAX);
                AdcParam->State = static_cast<uint8_t>(adc_read_opt.error().value());
                if (pOR->BreakEvent) {
                    SetEvent(pOR->BreakEvent);
                }
            }
        }
    }};

    return 0;
}

DC_LCARD_API
uint8_t __stdcall Fast_Adc(struct UsbLModul* pULM, struct ParamRead* AdcParam, struct OptionsRead* pOR)
{
    (void)pULM;
    (void)AdcParam;
    (void)pOR;

    return 101;  // Неподдерживаемая команда
}

DC_LCARD_API
uint8_t __stdcall SetTTL_Line(struct UsbLModul* pULM, uint16_t Line)
{
    assert(pULM);

    auto const modul_adapter = static_cast<ModulAdapter*>(pULM->Modul);
    assert(modul_adapter);

    const auto ttl_out_opt = modul_adapter->daq.TtlOut(Line);

    assert(!ttl_out_opt || (ttl_out_opt.error().value() <= UINT8_MAX));
    return ttl_out_opt ? 0 : static_cast<uint8_t>(ttl_out_opt.error().value());
}

DC_LCARD_API
uint8_t __stdcall GetTTL_Line(struct UsbLModul* pULM, uint16_t* TTLLine)
{
    assert(pULM);

    auto const modul_adapter = static_cast<ModulAdapter*>(pULM->Modul);
    assert(modul_adapter);

    const auto ttl_in_opt = modul_adapter->daq.TtlIn();
    if (!ttl_in_opt) {
        assert(ttl_in_opt.error().value() <= UINT8_MAX);
        return static_cast<uint8_t>(ttl_in_opt.error().value());
    }

    *TTLLine = ttl_in_opt.value();

    return 0;
}

DC_LCARD_API
void __stdcall ActiveTTL_Line(struct UsbLModul* pULM, bool Active)
{
    assert(pULM);

    auto const modul_adapter = static_cast<ModulAdapter*>(pULM->Modul);
    assert(modul_adapter);

    /*const auto ttl_enable_opt = */modul_adapter->daq.TtlEnable(Active);
}

DC_LCARD_API
bool __stdcall DAC_Exists(struct UsbLModul* aULM)
{
    (void)aULM;

    return false;
}

DC_LCARD_API
bool __stdcall DAC_Sample(struct UsbLModul* aULM, uint8_t aDACChannel, int16_t aValue)
{
    (void)aULM;
    (void)aDACChannel;
    (void)aValue;

    return false;
}
