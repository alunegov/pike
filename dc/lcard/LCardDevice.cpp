#include <LCardDevice.h>

#include <cassert>
#include <chrono>
#include <thread>

namespace ros { namespace dc { namespace lcard {

const char* const LCompName{"lcomp64.dll"};

LCardDevice::~LCardDevice()
{
    Deinit();
}

void LCardDevice::Init(ULONG slot_num)
{
    assert(device_ == nullptr);

    using CREATEFUNCPTR = IDaqLDevice*(*)(ULONG slot);

    ULONG status;

    lcomp_handle_ = LoadLibrary(LCompName);

    auto create_instance = (CREATEFUNCPTR)GetProcAddress(lcomp_handle_, "CreateInstance");
    
    IDaqLDevice* device_instance = create_instance(slot_num);

    HRESULT query_res = device_instance->QueryInterface(IID_ILDEV, (void**)&device_);

    device_instance->Release();
    //device_instance = nullptr;

    HANDLE device_handle = device_->OpenLDevice();

    SLOT_PAR slot_param;

    status = device_->GetSlotParam(&slot_param);

    board_type_ = slot_param.BoardType;

    const char* const biosName = DetectBiosName(board_type_);
    status = device_->LoadBios(const_cast<char*>(biosName));

    PLATA_DESCR_U2 plata_descr;

    status = device_->ReadPlataDescr(&plata_descr);

    adc_rate_params_ = DetectAdcRateParams(board_type_, plata_descr);
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
        FreeLibrary(lcomp_handle_);
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

    ttl_out_value = value;
}

void LCardDevice::TtlOut_SetPin(uint16_t value)
{
    const uint16_t new_ttl_out_value = ttl_out_value | (1u << value);

    return TtlOut(new_ttl_out_value);
}

void LCardDevice::TtlOut_ClrPin(uint16_t value)
{
    const uint16_t new_ttl_out_value = ttl_out_value & ~(1u << value);

    return TtlOut(new_ttl_out_value);
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

    const auto rate = GetRate(adc_rate_params_, reg_freq, channels.size(), 0.1);

    ADC_PAR adc_param;

    memset(&adc_param, 0, sizeof(adc_param));
    adc_param.t1.s_Type = L_ADC_PARAM;
    adc_param.t1.AutoInit = 1;
    adc_param.t1.dRate = adc_rate_params_.FClock / rate.first;
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

    void* data{nullptr};
    ULONG* sync{nullptr};

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
    size_t tmp_half_buffer{(half_buffer_count == 1) ? final_half_buffer : half_buffer};

    for (size_t i = 0; i < half_buffer_count; i++) {
        // ожидание заполнения очередной половины буфера
        while (f1 == f2) {
            f2 = (*sync < tmp_half_buffer) ? 0 : 1;
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        }

        int16_t* const values_tmp = values + half_buffer * i;
        const int16_t* const data_tmp = (int16_t*)data + half_buffer * f1;
        memmove(values_tmp, data_tmp, tmp_half_buffer * sizeof(int16_t));

        f1 = (*sync < half_buffer) ? 0 : 1;

        // для "последней половины" корректируем ожидаемое кол-во точек
        if ((i + 1) == half_buffer_count) {
            tmp_half_buffer = final_half_buffer;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }

    status = device_->StopLDevice();
}

const char* LCardDevice::DetectBiosName(ULONG board_type)
{
    switch (board_type) {
    case PCIA:
    case PCIB:
    case PCIC:
        return "todo";  // TODO: plata_descr.t1.BrdName
    case E440:
        return "e440";
    case E2010:
        return "e2010";
    case E2010B:
        return "e2010m";
    default:
        return "dummy";  // no bios needed
    }
}

AdcRateParams LCardDevice::DetectAdcRateParams(ULONG board_type, const PLATA_DESCR_U2& plata_descr)
{
    AdcRateParams res{0, 0, 0, 0, 0};

    switch (board_type) {
    case E440:
        res = {48000.0 / 2.0, 60, 65536, 1, 65500};
        break;
    case E140:
        if (plata_descr.t5.Rev == 'A') {
            res = {16000.0 / 2.0, 80, 65535, 1, 256};
        } else if (plata_descr.t5.Rev == 'B') {
            res = {16000.0 / 2.0, 40, 65535, 1, 256};
        } else {
            assert(false);
        }
        break;
    case E154:
        res = {48000.0 / 2.0, 10, 65530, 1, 65530};
        break;
    case E2010:
        res = {30000.0, 3, 30, 1, 255};
        break;
    case E2010B:
        res = {30000.0, 3, 30, 1, 65535};
        break;
    default:
        assert(false);
        break;
    }

    return res;
}

std::pair<uint32_t, uint16_t> LCardDevice::GetRate(const AdcRateParams& rateParams, double_t channelRate,
        size_t channelCount, double_t eps)
{
    assert(channelRate > 0);
    assert(channelCount > 0);
    assert(eps > 0);

    std::pair<uint32_t, uint16_t> res{rateParams.FClock_MinDiv, rateParams.IKD_MinKoeff};

    double_t dMinDelta{666666};
    bool bTmp{false};

    for (uint32_t i1 = rateParams.FClock_MinDiv; i1 <= rateParams.FClock_MaxDiv; i1++) {
        const double_t d2{rateParams.FClock / i1 / channelRate};
        
        size_t i2 = (size_t)trunc(d2) - (channelCount - 1u);
        if (i2 > rateParams.IKD_MaxKoeff) {
            continue;
        }
        if (i2 < rateParams.IKD_MinKoeff) {
            i2 = rateParams.IKD_MinKoeff;
        }
    
        const double_t d3{d2 - trunc(d2)};

        if (bTmp && (d3 > dMinDelta)) {
            break;
        }

        if ((d3 < dMinDelta) || (i2 == rateParams.IKD_MinKoeff)) {
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
