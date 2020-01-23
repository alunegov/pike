#include <LCardDevice.h>

#include <cassert>
#include <chrono>
#include <thread>

namespace ros { namespace dc { namespace lcard {

// TODO: use lcomp.dll for 32bit
const char* const LCompName{"lcomp64.dll"};

LCardDevice::~LCardDevice()
{
    NonVirtualDeinit();
}

tl::expected<void, std::error_code> LCardDevice::Init(size_t slot_num)
{
    assert(device_ == nullptr);

    using CREATEFUNCPTR = IDaqLDevice*(*)(ULONG slot);

    ULONG status;

    lcomp_handle_ = LoadLibrary(LCompName);
    if (lcomp_handle_ == nullptr) {
        Deinit();
        return tl::make_unexpected(std::make_error_code(std::errc::bad_address));
    }

    const auto create_instance = (CREATEFUNCPTR)GetProcAddress(lcomp_handle_, "CreateInstance");
    if (create_instance == nullptr) {
        Deinit();
        return tl::make_unexpected(std::make_error_code(std::errc::bad_address));
    }
    
    IDaqLDevice* const device_instance = create_instance(static_cast<ULONG>(slot_num));
    if (device_instance == nullptr) {
        Deinit();
        return tl::make_unexpected(std::make_error_code(std::errc::bad_address));
    }

    const HRESULT query_res = device_instance->QueryInterface(IID_ILDEV, (void**)&device_);
    if (!SUCCEEDED(query_res)) {
        // TODO: device_instance->Release()?
        Deinit();
        return tl::make_unexpected(std::make_error_code(std::errc::bad_address));
    }

    status = device_instance->Release();
    //device_instance = nullptr;

    const HANDLE device_handle = device_->OpenLDevice();
    if (device_handle == INVALID_HANDLE_VALUE) {
        Deinit();
        return tl::make_unexpected(std::make_error_code(std::errc::bad_address));
    }

    SLOT_PAR slot_param;

    status = device_->GetSlotParam(&slot_param);
    if (status != L_SUCCESS) {
        Deinit();
        return tl::make_unexpected(std::make_error_code(std::errc::bad_address));
    }

    board_type_ = slot_param.BoardType;

    const char* const biosName = DetectBiosName(board_type_);
    status = device_->LoadBios(const_cast<char*>(biosName));
    if ((status != L_SUCCESS) && (status != L_NOTSUPPORTED)) {
        Deinit();
        return tl::make_unexpected(std::make_error_code(std::errc::bad_address));
    }

    PLATA_DESCR_U2 plata_descr;

    status = device_->ReadPlataDescr(&plata_descr);
    if (status != L_SUCCESS) {
        Deinit();
        return tl::make_unexpected(std::make_error_code(std::errc::bad_address));
    }

    adc_rate_params_ = DetectAdcRateParams(board_type_, plata_descr);
    if (adc_rate_params_.FClock == 0) {
        Deinit();
        return tl::make_unexpected(std::make_error_code(std::errc::bad_address));
    }

    return {};
}

tl::expected<void, std::error_code> LCardDevice::Deinit()
{
    return NonVirtualDeinit();
}

tl::expected<void, std::error_code> LCardDevice::TtlEnable(bool enable)
{
    assert(device_ != nullptr);

    ASYNC_PAR async_param;

    async_param.s_Type = L_ASYNC_TTL_CFG;
    async_param.Mode = enable ? 1 : 0;

    const ULONG status = device_->IoAsync(&async_param);
    if (status != L_SUCCESS) {
        return tl::make_unexpected(std::make_error_code(std::errc::bad_address));
    }

    return {};
}

tl::expected<void, std::error_code> LCardDevice::TtlOut(uint16_t value)
{
    assert(device_ != nullptr);

    ASYNC_PAR async_param;

    async_param.s_Type = L_ASYNC_TTL_OUT;
    async_param.Data[0] = value;

    const ULONG status = device_->IoAsync(&async_param);
    if (status != L_SUCCESS) {
        return tl::make_unexpected(std::make_error_code(std::errc::bad_address));
    }

    ttl_out_value = value;

    return {};
}

tl::expected<void, std::error_code> LCardDevice::TtlOut_SetPin(uint16_t value)
{
    const uint16_t new_ttl_out_value = ttl_out_value | (1u << value);

    return TtlOut(new_ttl_out_value);
}

tl::expected<void, std::error_code> LCardDevice::TtlOut_ClrPin(uint16_t value)
{
    const uint16_t new_ttl_out_value = ttl_out_value & ~(1u << value);

    return TtlOut(new_ttl_out_value);
}

tl::expected<uint16_t, std::error_code> LCardDevice::TtlIn()
{
    assert(device_ != nullptr);

    ASYNC_PAR async_param;

    async_param.s_Type = L_ASYNC_TTL_INP;

    const ULONG status = device_->IoAsync(&async_param);
    if (status != L_SUCCESS) {
        return tl::make_unexpected(std::make_error_code(std::errc::bad_address));
    }

    return static_cast<uint16_t>(async_param.Data[0]);
}

// Задержка при ожидании заполнения половинки буфера при чтении
constexpr std::chrono::milliseconds AdcFillDelay{1};

tl::expected<void, std::error_code> LCardDevice::AdcRead(double_t& reg_freq, size_t point_count,
        const _Channels& channels, int16_t* values)
{
    assert(device_ != nullptr);

    assert(point_count > 0);
    assert((0 < channels.size()) && (channels.size() <= ULONG_MAX));
    assert(values != nullptr);

    ULONG status;

    size_t half_buffer{0};
    void* data{nullptr};
    ULONG* sync{nullptr};

    status = PrepareAdc(reg_freq, channels, &half_buffer, &data, &sync);
    if (status != L_SUCCESS) {
        return tl::make_unexpected(std::make_error_code(std::errc::bad_address));
    }

    // кол-во половинок, нужное для запрошенного количества точек
    size_t half_buffer_count = point_count * channels.size() / half_buffer;
    if (half_buffer * half_buffer_count < point_count * channels.size()) {
        half_buffer_count++;
    }
    assert(half_buffer * half_buffer_count >= point_count * channels.size());

    // размер "последней половины" - чтобы завершить чтение сразу после получения запрошенного количества точек (при
    // малой частоте сбора заполнение всей половины м.б. долгим).
    const size_t final_half_buffer = point_count * channels.size() - (half_buffer_count - 1) * half_buffer;

    status = device_->InitStartLDevice();
    if (status != L_SUCCESS) {
        return tl::make_unexpected(std::make_error_code(std::errc::bad_address));
    }

    status = device_->StartLDevice();
    if (status != L_SUCCESS) {
        return tl::make_unexpected(std::make_error_code(std::errc::bad_address));
    }

    //
    // с момента запуска, пока мы спим - *sync увеличивается

    size_t f1, f2;

    // какой смысл использовать InterlockedExchange(&s, *sync) (как в примере)? - ведь нужно синхронизировать доступ
    // к sync, а не к s.
    f1 = (*sync < half_buffer) ? 0 : 1;
    f2 = (*sync < half_buffer) ? 0 : 1;
    size_t tmp_half_buffer{(half_buffer_count == 1) ? final_half_buffer : half_buffer};

    for (size_t i = 0; i < half_buffer_count; i++) {
        // ожидание заполнения очередной половины буфера
        while (f1 == f2) {
            std::this_thread::sleep_for(AdcFillDelay);

            f2 = (*sync < tmp_half_buffer) ? 0 : 1;
        }

        int16_t* const values_tmp = values + half_buffer * i;
        const int16_t* const data_tmp = (int16_t*)data + half_buffer * f1;
        memmove(values_tmp, data_tmp, tmp_half_buffer * sizeof(int16_t));

        // для "последней половины" корректируем ожидаемое кол-во точек
        if ((i + 1) == half_buffer_count) {
            tmp_half_buffer = final_half_buffer;
        }

        std::this_thread::sleep_for(AdcFillDelay);

        f1 = (*sync < half_buffer) ? 0 : 1;
    }

    status = device_->StopLDevice();
    if (status != L_SUCCESS) {
        return tl::make_unexpected(std::make_error_code(std::errc::bad_address));
    }

    return {};
}

tl::expected<void, std::error_code> LCardDevice::AdcRead(double_t& reg_freq, const _Channels& channels,
        const std::atomic_bool& cancel_token, const std::function<AdcReadCallback>& callback)
{
    assert(device_ != nullptr);

    assert((0 < channels.size()) && (channels.size() <= ULONG_MAX));

    ULONG status;

    size_t half_buffer{0};
    void* data{nullptr};
    ULONG* sync{nullptr};

    // TODO: подгонять буфер/половину буфера под:
    // 1. определённое время наполнения (например, 400 мс):
    //    - если скорость наполнения низкая - уменьшать кол-во точек в шаге IrqStep или кол-во шагов Pages
    //    - если скорость наполнения высокая - накапливать точки в промежуточном буфере, внутри или снаружи
    // 2. кол-во точек кратное кол-ву каналов, чтобы во всех кадрах было целое кол-во каналов (точки всех каналов)

    status = PrepareAdc(reg_freq, channels, &half_buffer, &data, &sync);
    if (status != L_SUCCESS) {
        return tl::make_unexpected(std::make_error_code(std::errc::bad_address));
    }

    status = device_->InitStartLDevice();
    if (status != L_SUCCESS) {
        return tl::make_unexpected(std::make_error_code(std::errc::bad_address));
    }

    status = device_->StartLDevice();
    if (status != L_SUCCESS) {
        return tl::make_unexpected(std::make_error_code(std::errc::bad_address));
    }

    //
    // с момента запуска, пока мы спим - *sync увеличивается

    size_t f1, f2;

    // какой смысл использовать InterlockedExchange(&s, *sync) (как в примере)? - ведь нужно синхронизировать доступ
    // к sync, а не к s.
    f1 = (*sync < half_buffer) ? 0 : 1;
    f2 = (*sync < half_buffer) ? 0 : 1;

    while (true) {
        // ожидание заполнения очередной половины буфера или отмены чтения
        while ((f1 == f2) && !cancel_token) {
            std::this_thread::sleep_for(AdcFillDelay);

            f2 = (*sync < half_buffer) ? 0 : 1;
        }

        if (cancel_token) {
            break;
        }

        const int16_t* const data_tmp = (int16_t*)data + half_buffer * f1;
        callback(data_tmp, half_buffer);

        std::this_thread::sleep_for(AdcFillDelay);

        f1 = (*sync < half_buffer) ? 0 : 1;
    }

    status = device_->StopLDevice();
    if (status != L_SUCCESS) {
        return tl::make_unexpected(std::make_error_code(std::errc::bad_address));
    }

    return {};
}

tl::expected<void, std::error_code> LCardDevice::NonVirtualDeinit()
{
    if (device_ != nullptr) {
        ULONG status;

        status = device_->CloseLDevice();
        if (status != L_SUCCESS) {
            return tl::make_unexpected(std::make_error_code(std::errc::bad_address));
        }

        status = device_->Release();
        device_ = nullptr;  // по факту device_ уже удалён
    }

    if (lcomp_handle_ != nullptr) {
        const auto free_res = FreeLibrary(lcomp_handle_);
        if (!free_res) {
            return tl::make_unexpected(std::make_error_code(std::errc::bad_address));
        }
        lcomp_handle_ = nullptr;
    }

    return {};
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

ULONG LCardDevice::PrepareAdc(double_t& reg_freq, const _Channels& channels, size_t* half_buffer, void** data, ULONG** sync)
{
    assert(device_ != nullptr);
    assert((board_type_ == E440) || (board_type_ == E140) || (board_type_ == E154));  // adc_param.t1

    assert(reg_freq > 0);
    assert((0 < channels.size()) && (channels.size() <= ULONG_MAX));

    ULONG status;

    ULONG tm = 10000000;

    status = device_->RequestBufferStream(&tm, L_STREAM_ADC);
    if (status != L_SUCCESS) {
        return status;
    }

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
    if (status != L_SUCCESS) {
        return status;
    }

    const double_t old_reg_freq{reg_freq};
    reg_freq = 1 / ((channels.size() - 1) / adc_param.t1.dRate + adc_param.t1.dKadr);
    // большой уход выставленной от запрошенной частоты регистрации (частоты кадров)
    if (abs(reg_freq - old_reg_freq) > (0.02 * old_reg_freq)) {
        return 13;
    }

    status = device_->SetParametersStream(&adc_param.t1, &tm, data, (void**)sync, L_STREAM_ADC);
    if (status != L_SUCCESS) {
        return status;
    }

    // SetParametersStream могла откорректировать параметры буфера
    const ULONG irq_step = adc_param.t1.IrqStep; 
    const ULONG pages = adc_param.t1.Pages;

    // размер половины буфера платы, в отсчётах
    *half_buffer = irq_step * pages / 2;

    // размер отсчёта
    ULONG point_size;

    status = device_->GetParameter(L_POINT_SIZE, &point_size);
    if (status != L_SUCCESS) {
        return status;
    }

    assert(point_size == sizeof(int16_t));

    return status;
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
