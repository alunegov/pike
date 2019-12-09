#include <LCardDevice.h>

#include <cassert>
#include <chrono>
#include <thread>

namespace ros { namespace dc { namespace lcard {

constexpr char* LCompName{ "lcomp64.dll" };
constexpr ULONG SlotNum{ 0 };
constexpr char* BiosName{ "e440" };

// rev B - 140-М
const double_t FClock = 16000.0 / 2.0;
const uint16_t FClock_MinDiv = 40;  // макс. частота 200 КГц
const uint16_t FClock_MaxDiv = 65535;
const uint16_t IKD_MinKoeff = 1;
const uint16_t IKD_MaxKoeff = 256;

LCardDevice::~LCardDevice()
{
    Deinit();
}

void LCardDevice::Init()
{
    assert(device_ == nullptr);

    typedef IDaqLDevice* (*CREATEFUNCPTR)(ULONG slot);

    ULONG status;

    lcomp_handle_ = LoadLibrary(LCompName);

    CREATEFUNCPTR create_instance = (CREATEFUNCPTR)GetProcAddress(lcomp_handle_, "CreateInstance");
    
    IDaqLDevice* device_instance = create_instance(SlotNum);

    HRESULT query_res = device_instance->QueryInterface(IID_ILDEV, (void**)&device_);

    device_instance->Release();
    device_instance = nullptr;

    HANDLE device_handle = device_->OpenLDevice();

    SLOT_PAR slot_param;

    status = device_->GetSlotParam(&slot_param);

    board_type_ = slot_param.BoardType;

    status = device_->LoadBios(BiosName);

    PLATA_DESCR_U2 plata_descr;

    status = device_->ReadPlataDescr(&plata_descr);

    assert(plata_descr.t5.Quartz == FClock * 2 * 1000);
}

void LCardDevice::Deinit()
{
    if (device_ != nullptr) {
        ULONG status;

        status = device_->CloseLDevice();

        status = device_->Release();
        device_ = nullptr;
    }

    if (lcomp_handle_ != 0) {
        //FreeLibrary(lcomp_handle_);
        lcomp_handle_ = 0;
    }
}

void LCardDevice::TtlEnable(bool enable)
{
    assert(device_ != nullptr);

    ASYNC_PAR async_param;

    async_param.s_Type = L_ASYNC_TTL_CFG;
    async_param.Mode = enable ? 1 : 0;

    ULONG status = device_->IoAsync(&async_param);
}

void LCardDevice::TtlOut(uint16_t value)
{
    assert(device_ != nullptr);

    ASYNC_PAR async_param;

    async_param.s_Type = L_ASYNC_TTL_OUT;
    async_param.Data[0] = value;

    ULONG status = device_->IoAsync(&async_param);
}

uint16_t LCardDevice::TtlIn()
{
    assert(device_ != nullptr);

    ASYNC_PAR async_param;

    async_param.s_Type = L_ASYNC_TTL_INP;

    ULONG status = device_->IoAsync(&async_param);

    return static_cast<uint16_t>(async_param.Data[0]);
}

void LCardDevice::AdcRead(double_t& reg_freq, size_t point_count, const std::vector<uint16_t>& channels, int16_t* values)
{
    assert(device_ != nullptr);
    assert((board_type_ == E440) || (board_type_ == E140) || (board_type_ == E154));  // adc_param.t1

    assert(reg_freq > 0);
    assert(point_count > 0);
    assert((0 < channels.size()) && (channels.size() <= ULONG_MAX));
    assert(values != nullptr);

    ULONG status;

    ULONG tm = 10000000;

    status = device_->RequestBufferStream(&tm, L_STREAM_ADC);

    const auto rate = GetRate(reg_freq, channels.size(), 0.1);

    ADC_PAR adc_param;

    memset(&adc_param, 0, sizeof(adc_param));
    adc_param.t1.s_Type = L_ADC_PARAM;
    adc_param.t1.AutoInit = 1;
    adc_param.t1.dRate = FClock / rate.first;
    adc_param.t1.dKadr = rate.second / adc_param.t1.dRate;
    adc_param.t1.SynchroType = 3;
    if ((board_type_ == E440) || (board_type_ == E140) || (board_type_ == E154)) {
        adc_param.t1.SynchroType = 0;
    }
    adc_param.t1.NCh = static_cast<ULONG>(channels.size());
    for (size_t i = 0; i < channels.size(); i++) {
        adc_param.t1.Chn[i] = channels[i];
    }
    adc_param.t1.FIFO = 1024;
    adc_param.t1.IrqStep = 1024;
    adc_param.t1.Pages = 128;
    if ((board_type_ == E440) || (board_type_ == E140) || (board_type_ == E154)) {
        adc_param.t1.FIFO = 4096;
        adc_param.t1.IrqStep = 4096;
        adc_param.t1.Pages = 32;
    }
    adc_param.t1.IrqEna = 1;
    adc_param.t1.AdcEna = 1;

    status = device_->FillDAQparameters(&adc_param.t1);

    // большой уход выставленной от запрошенной частоты регистрации (частоты кадров)
    assert(abs(1 / ((channels.size() - 1) / adc_param.t1.dRate + adc_param.t1.dKadr) - reg_freq) < (0.02 * reg_freq));

    reg_freq = 1 / ((channels.size() - 1) / adc_param.t1.dRate + adc_param.t1.dKadr);

    void* data{ nullptr };
    ULONG* sync{ nullptr };

    status = device_->SetParametersStream(&adc_param.t1, &tm, &data, (void**)&sync, L_STREAM_ADC);

    // SetParametersStream могла откорректировать параметры буфера
    ULONG irq_step = adc_param.t1.IrqStep; 
    ULONG pages = adc_param.t1.Pages;

    ULONG point_size;

    device_->GetParameter(L_POINT_SIZE, &point_size);

    assert(point_size == sizeof(int16_t));

    // размер половины буфера платы, в отсчётах
    size_t half_buffer = irq_step * pages / 2;

    // кол-во половинок, нужное для запрошенного количества точек
    size_t half_buffer_count = point_count * channels.size() / half_buffer;
    if (half_buffer * half_buffer_count < point_count * channels.size()) {
        half_buffer_count++;
    }
    assert(half_buffer * half_buffer_count >= point_count * channels.size());

    // размер "последней половины" - чтобы завершить чтение сразу после получения запрошенного количества точек (при
    // малой частоте сбора заполнение всей половины м.б. долгим).
    size_t final_half_buffer = point_count * channels.size() - (half_buffer_count - 1) * half_buffer;

    status = device_->InitStartLDevice();

    status = device_->StartLDevice();

    //
    size_t f1, f2;
       
    // какой смысл использовать InterlockedExchange(&s, *sync) (как в примере)? - ведь нужно синхронизировать доступ
    // к sync, а не к s.
    f1 = (*sync < half_buffer) ? 0 : 1;
    f2 = (*sync < half_buffer) ? 0 : 1;
    size_t tmp_half_buffer{ (half_buffer_count == 1) ? final_half_buffer : half_buffer };

    for (size_t i = 0; i < half_buffer_count; i++) {
        // ожидание заполнения очередной половины буфера
        while (f1 == f2) {
            f2 = (*sync < tmp_half_buffer) ? 0 : 1;
            std::this_thread::sleep_for(std::chrono::milliseconds{ 1 });
        }

        int16_t* const values_tmp = values + half_buffer * i;
        const int16_t* const data_tmp = (int16_t*)data + half_buffer * f1;
        memmove(values_tmp, data_tmp, tmp_half_buffer * sizeof(int16_t));

        f1 = (*sync < half_buffer) ? 0 : 1;

        // для "последней половины" корректируем ожидаемое кол-во точек
        if ((i + 1) == half_buffer_count) {
            tmp_half_buffer = final_half_buffer;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds{ 1 });
    }

    status = device_->StopLDevice();
}

std::pair<uint16_t, uint16_t> LCardDevice::GetRate(double_t channelRate, size_t channelCount, double_t eps)
{
    assert(channelRate > 0);
    assert(channelCount > 0);
    assert(eps > 0);

    std::pair<uint16_t, uint16_t> res{ FClock_MinDiv, IKD_MinKoeff };

    double_t dMinDelta = 666666;
    bool bTmp = false;

    for (uint16_t i1 = FClock_MinDiv; i1 <= FClock_MaxDiv; i1++) {
        const double_t d2 = FClock / i1 / channelRate;
        
        size_t i2 = (size_t)trunc(d2) - (channelCount - 1u);
        if (i2 > IKD_MaxKoeff) {
            continue;
        }
        if (i2 < IKD_MinKoeff) {
            i2 = IKD_MinKoeff;
        }
    
        const double_t d3 = d2 - trunc(d2);

        if (bTmp && (d3 > dMinDelta)) {
            break;
        }

        if ((d3 < dMinDelta) || (i2 == IKD_MinKoeff)) {
            dMinDelta = d3;
            res.first = i1;
            res.second = static_cast<uint16_t>(i2);
        }

        if ((d3 < eps) && !bTmp) {
            bTmp = true;
        }
    }

    return res;
}

}}}
