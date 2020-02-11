#include <LCardDaq.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <thread>

#include <error_lcard.hpp>

namespace ros { namespace dc { namespace lcard {

LCardDaq::~LCardDaq()
{
    NonVirtualDeinit();
}

tl::expected<void, std::error_code> LCardDaq::Init(size_t slot_num)
{
    assert(device_ == nullptr);

    using CREATEFUNCPTR = IDaqLDevice*(*)(ULONG slot);

    lcomp_handle_ = LoadLibrary(LCompName);
    if (lcomp_handle_ == nullptr) {
        Deinit();
        return tl::make_unexpected(ros::make_error_code(ros::error_lcard::LoadLibraryErr));
    }

    const auto create_instance = reinterpret_cast<CREATEFUNCPTR>(GetProcAddress(lcomp_handle_, "CreateInstance"));
    if (create_instance == nullptr) {
        Deinit();
        return tl::make_unexpected(ros::make_error_code(ros::error_lcard::CreateInstanceAddrErr));
    }
    
    assert(slot_num <= ULONG_MAX);
    IDaqLDevice* const device_instance = create_instance(static_cast<ULONG>(slot_num));
    if (device_instance == nullptr) {
        Deinit();
        return tl::make_unexpected(ros::make_error_code(ros::error_lcard::CreateInstanceErr));
    }

    const HRESULT query_res = device_instance->QueryInterface(IID_ILDEV, (void**)&device_);
    if (!SUCCEEDED(query_res)) {
        // TODO: device_instance->Release()?
        Deinit();
        return tl::make_unexpected(ros::make_error_code(ros::error_lcard::QueryInterfaceErr));
    }

    ULONG status = device_instance->Release();
    //device_instance = nullptr;

    const HANDLE device_handle = device_->OpenLDevice();
    if (device_handle == INVALID_HANDLE_VALUE) {
        Deinit();
        return tl::make_unexpected(ros::make_error_code(ros::error_lcard::OpenLDeviceErr));
    }

    SLOT_PAR slot_param;

    status = device_->GetSlotParam(&slot_param);
    if (status != L_SUCCESS) {
        Deinit();
        return tl::make_unexpected(ros::make_error_code(ros::error_lcard::GetSlotParamErr));
    }

    board_type_ = slot_param.BoardType;

    const char* const biosName = DetectBiosName(board_type_);
    status = device_->LoadBios(const_cast<char*>(biosName));
    if ((status != L_SUCCESS) && (status != L_NOTSUPPORTED)) {
        Deinit();
        return tl::make_unexpected(ros::make_error_code(ros::error_lcard::LoadBiosErr));
    }

    PLATA_DESCR_U2 plata_descr;

    status = device_->ReadPlataDescr(&plata_descr);
    if (status != L_SUCCESS) {
        Deinit();
        return tl::make_unexpected(ros::make_error_code(ros::error_lcard::ReadPlataDescrErr));
    }

    status = device_->EnableCorrection();
    if ((status != L_SUCCESS) && (status != L_NOTSUPPORTED)) {
        Deinit();
        return tl::make_unexpected(ros::make_error_code(ros::error_lcard::EnableCorrectionErr));
    }

    adc_rate_params_ = DetectAdcRateParams(board_type_, plata_descr);
    if (adc_rate_params_.FClock == 0) {
        Deinit();
        return tl::make_unexpected(ros::make_error_code(ros::error_lcard::DetectAdcRateParamsErr));
    }

    return {};
}

tl::expected<void, std::error_code> LCardDaq::Deinit()
{
    return NonVirtualDeinit();
}

tl::expected<void, std::error_code> LCardDaq::TtlEnable(bool enable)
{
    assert(device_ != nullptr);

    ASYNC_PAR async_param;

    async_param.s_Type = L_ASYNC_TTL_CFG;
    async_param.Mode = enable ? 1 : 0;

    const ULONG status = device_->IoAsync(&async_param);
    if (status != L_SUCCESS) {
        return tl::make_unexpected(ros::make_error_code(ros::error_lcard::IoAsyncErr));
    }

    return {};
}

tl::expected<void, std::error_code> LCardDaq::TtlOut(uint16_t value)
{
    assert(device_ != nullptr);

    ASYNC_PAR async_param;

    async_param.s_Type = L_ASYNC_TTL_OUT;
    async_param.Data[0] = value;

    const ULONG status = device_->IoAsync(&async_param);
    if (status != L_SUCCESS) {
        return tl::make_unexpected(ros::make_error_code(ros::error_lcard::IoAsyncErr));
    }

    ttl_out_value = value;

    return {};
}

tl::expected<void, std::error_code> LCardDaq::TtlOut_SetPin(uint16_t value)
{
    const uint16_t new_ttl_out_value = ttl_out_value | (1u << value);

    return TtlOut(new_ttl_out_value);
}

tl::expected<void, std::error_code> LCardDaq::TtlOut_ClrPin(uint16_t value)
{
    const uint16_t new_ttl_out_value = ttl_out_value & ~(1u << value);

    return TtlOut(new_ttl_out_value);
}

tl::expected<uint16_t, std::error_code> LCardDaq::TtlIn()
{
    assert(device_ != nullptr);

    ASYNC_PAR async_param;

    async_param.s_Type = L_ASYNC_TTL_INP;

    const ULONG status = device_->IoAsync(&async_param);
    if (status != L_SUCCESS) {
        return tl::make_unexpected(ros::make_error_code(ros::error_lcard::IoAsyncErr));
    }

    return static_cast<uint16_t>(async_param.Data[0]);
}

tl::expected<void, std::error_code> LCardDaq::AdcRead(double_t& reg_freq, size_t points_count,
        const _Channels& channels, int16_t* values)
{
    assert(device_ != nullptr);

    assert(points_count > 0);
    assert(!channels.empty());
    assert(values != nullptr);

    std::atomic_bool cancel_token{false};

    int16_t* vals{values};
    const int16_t* const vals_end{values + points_count * channels.size()};
    // capture by ref are safe here as latter AdcRead is synchronous
    const auto callback = [&vals, vals_end, &cancel_token](const int16_t* values, size_t values_count) {
        assert(values != nullptr);
        assert(values_count > 0);

        const auto cur_values_count = ((vals + values_count) <= vals_end) ? values_count : vals_end - vals;
        assert(cur_values_count > 0);
        assert(cur_values_count <= values_count);

        memmove(vals, values, cur_values_count);

        vals += cur_values_count;

        if (vals >= vals_end) {
            cancel_token = true;
        }
    };

    return AdcRead(reg_freq, channels, callback, SyncAdcReadCallbackInterval, cancel_token);
}

tl::expected<void, std::error_code> LCardDaq::AdcRead(double_t& reg_freq, const _Channels& channels,
        const std::function<AdcReadCallback>& callback, const std::chrono::milliseconds& callback_interval,
        const std::atomic_bool& cancel_token)
{
    assert(device_ != nullptr);

    size_t half_buffer{0};
    void* data{nullptr};
    ULONG* sync{nullptr};

    const auto prepare_adc_opt = PrepareAdc(reg_freq, channels, callback_interval, half_buffer, &data, &sync);
    if (!prepare_adc_opt) {
        return tl::make_unexpected(prepare_adc_opt.error());
    }

    ULONG status = device_->InitStartLDevice();
    if (status != L_SUCCESS) {
        return tl::make_unexpected(ros::make_error_code(ros::error_lcard::InitStartLDeviceErr));
    }

    status = device_->StartLDevice();
    if (status != L_SUCCESS) {
        return tl::make_unexpected(ros::make_error_code(ros::error_lcard::StartLDeviceErr));
    }

    //
    // с момента запуска, пока мы спим - *sync увеличивается

    // какой смысл использовать InterlockedExchange(&s, *sync) (как в примере)? - ведь нужно синхронизировать доступ
    // к sync, а не к s.
    size_t f1 = (*sync < half_buffer) ? 0 : 1;
    size_t f2 = (*sync < half_buffer) ? 0 : 1;

    while (true) {
        // ожидание заполнения очередной половины буфера или отмены чтения
        // TODO: таймаут ожидания заполнения буфера (иначе как узнать про физическое отключение платы и другие ошибки чтения)
        while ((f1 == f2) && !cancel_token) {
            std::this_thread::sleep_for(AdcFillDelay);

            f2 = (*sync < half_buffer) ? 0 : 1;
        }
        if (cancel_token) {
            break;
        }

        const int16_t* const data_tmp = static_cast<int16_t*>(data) + half_buffer * f1;
        callback(data_tmp, half_buffer);

        std::this_thread::sleep_for(AdcFillDelay);

        f1 = (*sync < half_buffer) ? 0 : 1;
    }

    status = device_->StopLDevice();
    if (status != L_SUCCESS) {
        return tl::make_unexpected(ros::make_error_code(ros::error_lcard::StopLDeviceErr));
    }

    return {};
}

tl::expected<void, std::error_code> LCardDaq::NonVirtualDeinit()
{
    if (device_ != nullptr) {
        ULONG status = device_->CloseLDevice();
        if (status != L_SUCCESS) {
            return tl::make_unexpected(ros::make_error_code(ros::error_lcard::CloseLDeviceErr));
        }

        status = device_->Release();
        device_ = nullptr;  // по факту device_ уже удалён
    }

    if (lcomp_handle_ != nullptr) {
        const auto free_res = FreeLibrary(lcomp_handle_);
        if (!free_res) {
            return tl::make_unexpected(ros::make_error_code(ros::error_lcard::FreeLibraryErr));
        }
        lcomp_handle_ = nullptr;
    }

    return {};
}

const char* LCardDaq::DetectBiosName(ULONG board_type)
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

AdcRateParams LCardDaq::DetectAdcRateParams(ULONG board_type, const PLATA_DESCR_U2& plata_descr)
{
    AdcRateParams res{};

    switch (board_type) {
    case E440:
        res = {48'000.0 / 2.0, 60, 65'536, 1, 65'500, 32, 64'000};
        break;
    case E140:
        if (plata_descr.t5.Rev == 'A') {
            res = {16'000.0 / 2.0, 80, 65'535, 1, 256, 32, 64'000};
        } else if (plata_descr.t5.Rev == 'B') {
            res = {16'000.0 / 2.0, 40, 65'535, 1, 256, 32, 64'000};
        } else {
            assert(false);
        }
        break;
    case E154:
        res = {48'000.0 / 2.0, 10, 65'530, 1, 65'530, 32, 64'000};
        break;
    case E2010:
        res = {30'000.0, 3, 30, 1, 255, 32, 1'000'000};
        break;
    case E2010B:
        res = {30'000.0, 3, 30, 1, 65'535, 32, 1'000'000};
        break;
    default:
        assert(false);
        break;
    }

    return res;
}

tl::expected<void, std::error_code> LCardDaq::PrepareAdc(double_t& reg_freq, const _Channels& channels,
        const std::chrono::milliseconds& tick_interval, size_t& half_buffer, void** data, ULONG** sync)
{
    assert(device_ != nullptr);
    assert((board_type_ == PCIA) || (board_type_ == PCIB) || (board_type_ == PCIC)
            || (board_type_ == E440) || (board_type_ == E140) || (board_type_ == E154));  // adc_param.t1

    assert(reg_freq > 0);
    assert(!channels.empty());

    // TODO: хватит ли 10 МБ для 2010?
    ULONG tm = 10'000'000;

    ULONG status = device_->RequestBufferStream(&tm, L_STREAM_ADC);
    if (status != L_SUCCESS) {
        return tl::make_unexpected(ros::make_error_code(ros::error_lcard::RequestBufferStreamErr));
    }

    ADC_PAR adc_param;

    memset(&adc_param, 0, sizeof(adc_param));
    adc_param.t1.s_Type = L_ADC_PARAM;
    adc_param.t1.AutoInit = 1;
    const auto rate = SelectRate(adc_rate_params_, reg_freq, channels.size(), 0.1);
    adc_param.t1.dRate = adc_rate_params_.FClock / rate.FClock_Div;
    adc_param.t1.dKadr = rate.IKD_Koeff / adc_param.t1.dRate;
    if ((board_type_ == E440) || (board_type_ == E140) || (board_type_ == E154)) {
        adc_param.t1.SynchroType = 0;
    } else {
        adc_param.t1.SynchroType = 3;
    }
    assert(channels.size() <= ULONG_MAX);
    adc_param.t1.NCh = static_cast<ULONG>(channels.size());
    assert(channels.size() <= std::size(adc_param.t1.Chn));
    std::copy(channels.begin(), channels.end(), std::begin(adc_param.t1.Chn));
    const auto adc_buffer = SelectAdcBuffer(adc_rate_params_, reg_freq, channels.size(), tick_interval);
    adc_param.t1.FIFO = adc_buffer.FIFO;
    adc_param.t1.IrqStep = adc_buffer.IrqStep;
    adc_param.t1.Pages = adc_buffer.Pages;
    adc_param.t1.IrqEna = 1;
    adc_param.t1.AdcEna = 1;

    // может откорректировать dRate, dKadr, FIFO, IrqStep и NCh в adc_param
    status = device_->FillDAQparameters(&adc_param.t1);
    if (status != L_SUCCESS) {
        return tl::make_unexpected(ros::make_error_code(ros::error_lcard::FillDAQparametersErr));
    }
    assert(adc_param.t1.NCh == static_cast<ULONG>(channels.size()));

    // может откорректировать FIFO, IrqStep и Pages в adc_param
    status = device_->SetParametersStream(&adc_param.t1, &tm, data, (void**)sync, L_STREAM_ADC);
    if (status != L_SUCCESS) {
        return tl::make_unexpected(ros::make_error_code(ros::error_lcard::SetParametersStreamErr));
    }
    assert(adc_param.t1.IrqStep * adc_param.t1.Pages == tm);

    const double_t old_reg_freq{reg_freq};
    reg_freq = 1 / ((channels.size() - 1) / adc_param.t1.dRate + adc_param.t1.dKadr);
    // большой уход выставленной от запрошенной частоты регистрации (частоты кадров)
    if (abs(reg_freq - old_reg_freq) > (0.02 * old_reg_freq)) {
        return tl::make_unexpected(ros::make_error_code(ros::error_lcard::SetRegFreqErr));
    }

    // размер половины буфера платы, в отсчётах
    const ULONG irq_step = adc_param.t1.IrqStep;
    const ULONG pages = adc_param.t1.Pages;
    half_buffer = irq_step * pages / 2;
    assert(half_buffer * 2 == irq_step * pages);

    // чтобы в последнем кадре были все каналы
    if (half_buffer % channels.size() != 0) {
        return tl::make_unexpected(ros::make_error_code(ros::error_lcard::NotFullCadrInHalfBuffer));
    }

    // размер отсчёта
    ULONG point_size;
    status = device_->GetParameter(L_POINT_SIZE, &point_size);
    if (status != L_SUCCESS) {
        return tl::make_unexpected(ros::make_error_code(ros::error_lcard::GetParameterErr));
    }
    assert(point_size == sizeof(int16_t));

    return {};
}

LCardDaq::RateParams LCardDaq::SelectRate(const AdcRateParams& rateParams, double_t channelRate, size_t channelCount,
        double_t eps)
{
    assert(channelRate > 0);
    assert(channelCount > 0);
    assert(eps > 0);

    LCardDaq::RateParams res{rateParams.FClock_MinDiv, rateParams.IKD_MinKoeff};

    double_t dMinDelta{666'666};
    bool bTmp{false};

    for (uint32_t i1 = rateParams.FClock_MinDiv; i1 <= rateParams.FClock_MaxDiv; i1++) {
        const double_t d2{rateParams.FClock / i1 / channelRate};
        
        size_t i2 = static_cast<size_t>(trunc(d2)) - (channelCount - 1u);
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
            res.FClock_Div = i1; 
            assert(i2 <= UINT16_MAX);
            res.IKD_Koeff = static_cast<uint16_t>(i2);
        }

        if ((d3 < eps) && !bTmp) {
            bTmp = true;
        }
    }

    return res;
}

// в примере было для E440/E140/E154 - 4096/4096/32, для PCIA/PCIB/PCIC - 1024/1024/128
LCardDaq::AdcBufferParams LCardDaq::SelectAdcBuffer(const AdcRateParams& rateParams, double_t channelRate,
        size_t channelCount, const std::chrono::milliseconds& tick_interval)
{
    assert(channelRate > 0);
    assert(channelCount > 0);
    assert(tick_interval.count() > 0);

    // TODO: НОД rateParams.IrqStep_Min и channelCount для irq_step
    assert(rateParams.IrqStep_Min * channelCount <= ULONG_MAX);
    LCardDaq::AdcBufferParams res{4096, static_cast<ULONG>(rateParams.IrqStep_Min * channelCount), 2};

    auto points_in_tick{static_cast<size_t>(channelRate * tick_interval.count() * channelCount)};
    // слишком маленький полу-буфер - делаем хотя бы на кадр
    if (points_in_tick == 0) {
        points_in_tick = channelCount;
    }

    double_t min_delta{6'666'666};

    // irq_step д.б. кратен 32
    for (size_t irq_step = rateParams.IrqStep_Min; irq_step <= rateParams.IrqStep_Max; irq_step += 32) {
        const auto pages = 2 * points_in_tick / irq_step;
        if (pages < 16) {
            continue;
        }

        const auto half_buffer = irq_step * pages / 2;
        if (half_buffer % channelCount != 0) {
            continue;
        }

        if (std::abs((double_t)half_buffer - points_in_tick) <= min_delta) {
            min_delta = std::abs((double_t)half_buffer - points_in_tick);
            assert(irq_step <= ULONG_MAX);
            res.IrqStep = static_cast<ULONG>(irq_step);
            assert(pages <= ULONG_MAX);
            res.Pages = static_cast<ULONG>(pages);
        }
    }

    return res;
}

}}}
