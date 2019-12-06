#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <iomanip>
#include <iostream>
#include <vector>
#include <thread>

#include <LCardDevice.h>

template <typename T>
T calc_max(const T* data, size_t count, size_t channel_index, size_t channelsCount)
{
    const T* const data_end = data + count * channelsCount;

    const T* tmp = data + channel_index;

    T max = *tmp;
    tmp += channelsCount;

    while (tmp < data_end) {
        if (max < *tmp) {
            max = *tmp;
        }
        tmp += channelsCount;
    }

    return max;
}

template <typename T>
double_t calc_mean(const T* data, size_t count, size_t channel_index, size_t channelsCount)
{
    T sum{ 0 };
    for (size_t i = 0; i < count; i++) {
        sum += data[i * channelsCount + channel_index];
    }
    return static_cast<double_t>(sum) / count;
}

template <typename T>
double_t calc_rms(const T* data, size_t count, size_t channel_index, size_t channelsCount)
{
    const double_t mean = calc_mean(data, count, channel_index, channelsCount);

    double_t sqr_sum{ 0 };
    for (size_t i = 0; i < count; i++) {
        double_t val_wo_dc = data[i * channelsCount + channel_index] - mean;

        sqr_sum += val_wo_dc * val_wo_dc;
    }
    return std::sqrt(sqr_sum / count);
}

int main(int, const char**)
{
    constexpr uint32_t PointsCount{ 32768 };

    const std::vector<uint16_t> Channels{
        0 | 32,
        1 | 32,
    };

    ros::dc::lcard::LCardDevice dev;

    dev.Init();

    std::vector<int16_t> values(PointsCount * Channels.size());

    for (uint_fast8_t j = 0; j < 5; j++) {
        double_t reg_freq{ 12.8 };

        auto read_future = std::async(std::launch::async, [&]() {
            dev.AdcRead(reg_freq, PointsCount, Channels, values.data());
        });

        read_future.wait();

        // значения АЦП -> Вольты
        // TODO: учёт усиления
        const double_t AdcToVolt{ 10.0 / 8000 };

        /*for (auto& v : values) {
            v *= AdcToVolt;
        }*/

        // точки
        const double_t dt{ 1.0 / (reg_freq * 1000) };

        for (size_t i = 0; i < values.size() / Channels.size(); i++) {
            //std::cout << i * dt << ";" << values[2 * i + 0] * AdcToVolt << ";" << values[2 * i + 1] * AdcToVolt << std::endl;
        }

        // скз
        const auto rms_ch1 = calc_rms(values.data(), values.size() / Channels.size(), 0, Channels.size());
        const auto rms_ch2 = calc_rms(values.data(), values.size() / Channels.size(), 1, Channels.size());
        std::cout << rms_ch1 * AdcToVolt << ";" << rms_ch2 * AdcToVolt << std::endl;
    }

    dev.Deinit();

    return 0;
}

/*int main2(int, const char**)
{
    ULONG status;

   /*std::thread th{[]() {
       ULONG halfbuffer = IrqStep*pages/2;              // Собираем половинками кольцевого буфера
       std::atomic<ULONG> s;
       ULONG f1, f2;

       while (sync == nullptr) {
           Sleep(0);
       }
       
       s.exchange(*sync);
       f1 = (s < halfbuffer) ? 0 : 1;
       f2 = (s < halfbuffer) ? 0 : 1;

       for (size_t i = 0; i < 4; i++) {
           while (f1 == f2) {
               s.exchange(*sync);
               f2 = (s < halfbuffer) ? 0 : 1;
           }

            s.exchange(*sync);
            f1 = (s < halfbuffer) ? 0 : 1;

            std::cout << s << std::endl;

            std::this_thread::yield();
       }
   }};*//*

    std::vector<int16_t> dd;
    std::vector<uint16_t> ttl_in;

    HINSTANCE lib = LoadLibrary("lcomp64.dll");
    CREATEFUNCPTR create_instance = (CREATEFUNCPTR)GetProcAddress(lib, "CreateInstance");
    
    IDaqLDevice* device = create_instance(0);
    if(device == NULL) { std::cout << "FAILED  -> CreateInstance" << std::endl; return 1; }
    std::cout << "SUCCESS -> CreateInstance" << std::endl;

    IDaqLDevice* pI;
    HRESULT hr = device->QueryInterface(IID_ILDEV, (void**)&pI);
    if(!SUCCEEDED(hr)) { std::cout << "FAILED  -> QueryInterface" << std::endl; return 1; }
    std::cout << "SUCCESS -> QueryInterface" << std::endl;

    status = device->Release();
    M_OK("Release IUnknown",std::endl);
    std::cout << ".......... Ref: " << status << std::endl;

    HANDLE hVxd = pI->OpenLDevice();
    if(hVxd == INVALID_HANDLE_VALUE) { M_FAIL("OpenLDevice",hVxd); goto end; } else M_OK("OpenLDevice",std::endl);
    std::cout << ".......... HANDLE: " << std::hex << hVxd << std::endl;

    SLOT_PAR sl;
    status = pI->GetSlotParam(&sl);
    if(status != L_SUCCESS) { M_FAIL("GetSlotParam",status); goto end; } else M_OK("GetSlotParam",std::endl);

    std::cout << ".......... Type    " << sl.BoardType << std::endl;
    std::cout << ".......... DSPType " << sl.DSPType << std::endl;
    std::cout << ".......... InPipe MTS" << sl.Dma << std::endl;
    std::cout << ".......... OutPipe MTS" << sl.DmaDac << std::endl;

    status = pI->LoadBios("e440");
    if((status != L_SUCCESS) && (status!=L_NOTSUPPORTED)) { M_FAIL("LoadBios",status); goto end; } else M_OK("LoadBios",std::endl);

    PLATA_DESCR_U2 pd;
    status = pI->ReadPlataDescr(&pd);
    // ОБЯЗАТЕЛЬНО ДЕЛАТЬ! (иначе расчеты параметров сбора данных невозможны тк нужна информация о названии модуля и частоте кварца )
    if(status != L_SUCCESS) { M_FAIL("ReadPlataDescr",status); goto end; } else M_OK("ReadPlataDescr",std::endl);

    switch (sl.BoardType)
    {
    case PCIA:
    case PCIB:
    case PCIC:  {
                    std::cout << ".......... SerNum       " << pd.t1.SerNum << std::endl;
                    std::cout << ".......... BrdName      " << pd.t1.BrdName << std::endl;
                    std::cout << ".......... Rev          " << pd.t1.Rev << std::endl;
                    std::cout << ".......... DspType      " << pd.t1.DspType << std::endl;
                    std::cout << ".......... IsDacPresent " << pd.t1.IsDacPresent << std::endl;
                    std::cout << ".......... Quartz       " << std::dec << pd.t1.Quartz << std::endl;
                } break;

    case E140:  {
                    std::cout << ".......... SerNum       " << pd.t5.SerNum << std::endl;
                    std::cout << ".......... BrdName      " << pd.t5.BrdName << std::endl;
                    std::cout << ".......... Rev          " << pd.t5.Rev << std::endl;
                    std::cout << ".......... DspType      " << pd.t5.DspType << std::endl;
                    std::cout << ".......... IsDacPresent " << pd.t5.IsDacPresent << std::endl;
                    std::cout << ".......... Quartz       " << std::dec << pd.t5.Quartz << std::endl;
                } break;

    case E440:  {
                    std::cout << ".......... SerNum       " << pd.t4.SerNum << std::endl;
                    std::cout << ".......... BrdName      " << pd.t4.BrdName << std::endl;
                    std::cout << ".......... Rev          " << pd.t4.Rev << std::endl;
                    std::cout << ".......... DspType      " << pd.t4.DspType << std::endl;
                    std::cout << ".......... IsDacPresent " << pd.t4.IsDacPresent << std::endl;
                    std::cout << ".......... Quartz       " << std::dec << pd.t4.Quartz << std::endl;
                } break;
    case E2010B:
    case E2010: {
                    std::cout << ".......... SerNum       " << pd.t6.SerNum << std::endl;
                    std::cout << ".......... BrdName      " << pd.t6.BrdName << std::endl;
                    std::cout << ".......... Rev          " << pd.t6.Rev << std::endl;
                    std::cout << ".......... DspType      " << pd.t6.DspType << std::endl;
                    std::cout << ".......... IsDacPresent " << pd.t6.IsDacPresent << std::endl;
                    std::cout << ".......... Quartz       " << std::dec << pd.t6.Quartz << std::endl;
                } break;
    case E154:  {
                    std::cout << ".......... SerNum       " << pd.t7.SerNum << std::endl;
                    std::cout << ".......... BrdName      " << pd.t7.BrdName << std::endl;
                    std::cout << ".......... Rev          " << pd.t7.Rev << std::endl;
                    std::cout << ".......... DspType      " << pd.t7.DspType << std::endl;
                    std::cout << ".......... IsDacPresent " << pd.t7.IsDacPresent << std::endl;
                    std::cout << ".......... Quartz       " << std::dec << pd.t7.Quartz << std::endl;
                } break;
    case L791:  {
                    std::cout << ".......... SerNum       " << pd.t3.SerNum << std::endl;
                    std::cout << ".......... BrdName      " << pd.t3.BrdName << std::endl;
                    std::cout << ".......... Rev          " << pd.t3.Rev << std::endl;
                    std::cout << ".......... DspType      " << pd.t3.DspType << std::endl;
                    std::cout << ".......... IsDacPresent " << pd.t3.IsDacPresent << std::endl;
                    std::cout << ".......... Quartz       " << std::dec << pd.t3.Quartz << std::endl;
                } break;
    }

    ASYNC_PAR pp;

    pp.s_Type = L_ASYNC_TTL_CFG;
    pp.Mode = 1;
    status = pI->IoAsync(&pp);
    if(status != L_SUCCESS) M_FAIL("IoAsync TTL_CFG",status); else M_OK("IoAsync TTL_CFG",std::endl);

    auto t1 = std::chrono::high_resolution_clock::now();

    pp.s_Type = L_ASYNC_TTL_INP;
    for (size_t i = 0; i < 1000; i++) {
        //std::this_thread::sleep_for(std::chrono::milliseconds{33});
        status = pI->IoAsync(&pp);
        if (status != L_SUCCESS) {
            std::cout << ".......... TTL_IN: FAILED" << std::endl;
            continue;
        }
        //std::cout << i << ";" << (pp.Data[0] & 0x1) << ";" << (pp.Data[0] & 0x2) << ";" << (pp.Data[0] & 0x4) << ";" << (pp.Data[0] & 0x8) << std::endl;
        //ttl_in.emplace_back(static_cast<uint16_t>(pp.Data[0] & 0x1));
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << std::endl;

    t1 = std::chrono::high_resolution_clock::now();

    pp.s_Type = L_ASYNC_TTL_OUT;
    for (size_t i = 0; i < 1000; i++) {
        //std::this_thread::sleep_for(std::chrono::milliseconds{33});
        pp.Data[0] = 0xffc0;//0xa525;
        status = pI->IoAsync(&pp);
        if (status != L_SUCCESS) {
            std::cout << ".......... TTL_OUT: FAILED" << std::endl;
            continue;
        }
        //std::cout << i << ";" << (pp.Data[0] & 0x1) << ";" << (pp.Data[0] & 0x2) << ";" << (pp.Data[0] & 0x4) << ";" << (pp.Data[0] & 0x8) << std::endl;
        //ttl_in.emplace_back(static_cast<uint16_t>(pp.Data[0] & 0x1));
    }

    t2 = std::chrono::high_resolution_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << std::endl;

    //goto end;

    pp.s_Type = L_ASYNC_TTL_OUT;
    pp.Data[0] = 0xa525;
    status = pI->IoAsync(&pp);
    std::cout << ".......... TTL_OUT: " << std::hex << pp.Data[0] << (status ? " FAILED":" SUCCESS");
    std::this_thread::sleep_for(std::chrono::milliseconds{1333});

    std::cout << std::dec;

   DWORD tm = 10000000;
   status = pI->RequestBufferStream(&tm,L_STREAM_ADC);
   if(status != L_SUCCESS) { M_FAIL("RequestBufferStream(ADC)",status); goto end; } else M_OK("RequestBufferStream(ADC)",std::endl);
   std::cout << ".......... Allocated memory size(word): " << tm << std::endl;

   ADC_PAR adcPar;
         adcPar.t1.s_Type = L_ADC_PARAM;
         adcPar.t1.AutoInit = 1;
         adcPar.t1.dRate = 210.0;
         adcPar.t1.dKadr = 0;
         adcPar.t1.dScale = 0;
         adcPar.t1.SynchroType = 3; //3
         if(sl.BoardType==E440 || sl.BoardType==E140 || sl.BoardType==E154) adcPar.t1.SynchroType = 0;//0
         adcPar.t1.SynchroSensitivity = 0;
         adcPar.t1.SynchroMode = 0;
         adcPar.t1.AdChannel = 0;
         adcPar.t1.AdPorog = 0;
         adcPar.t1.NCh = 2;
         adcPar.t1.Chn[0] = 0x0 | 0x20;
         adcPar.t1.Chn[1] = 0x7 | 0x20;
         adcPar.t1.Chn[2] = 0x2;
         adcPar.t1.Chn[3] = 0x3;
         adcPar.t1.FIFO = 1024;
         adcPar.t1.IrqStep = 1024;
         adcPar.t1.Pages = 128;
         if(sl.BoardType==E440 || sl.BoardType==E140 || sl.BoardType==E154)
         {
            adcPar.t1.FIFO = 4096;
            adcPar.t1.IrqStep = 4096;
            adcPar.t1.Pages = 32;
         }
         adcPar.t1.IrqEna = 1;
         adcPar.t1.AdcEna = 1;

         status = pI->FillDAQparameters(&adcPar.t1);
         if(status != L_SUCCESS) { M_FAIL("FillDAQparameters(ADC)",status); goto end; } else M_OK("FillDAQparameters(ADC)",std::endl);

         std::cout << ".......... Buffer size(word):      " << tm << std::endl;
         std::cout << ".......... Pages:                  " << adcPar.t1.Pages << std::endl;
         std::cout << ".......... IrqStep:                " << adcPar.t1.IrqStep << std::endl;
         std::cout << ".......... FIFO:                   " << adcPar.t1.FIFO << std::endl;
         std::cout << ".......... Rate:                   " << adcPar.t1.dRate << std::endl;
         std::cout << ".......... Kadr:                   " << adcPar.t1.dKadr << std::endl << std::endl;

         status = pI->SetParametersStream(&adcPar.t1, &tm, (void **)&data, (void **)&sync,L_STREAM_ADC);
         if(status != L_SUCCESS) { M_FAIL("SetParametersStream(ADC)",status); goto end; } else M_OK("SetParametersStream(ADC)",std::endl);

         std::cout << ".......... Used buffer size(points): " << tm << std::endl;
         std::cout << ".......... Pages:                  " << adcPar.t1.Pages << std::endl;
         std::cout << ".......... IrqStep:                " << adcPar.t1.IrqStep << std::endl;
         std::cout << ".......... FIFO:                   " << adcPar.t1.FIFO << std::endl;
         std::cout << ".......... Rate:                   " << adcPar.t1.dRate << std::endl;
         std::cout << ".......... Kadr:                   " << adcPar.t1.dKadr << std::endl << std::endl;

         IrqStep = adcPar.t1.IrqStep; // обновили глобальные переменные котрые потом используются в ServiceThread
         pages = adcPar.t1.Pages;

   pI->GetParameter(L_POINT_SIZE, &pointsize);
   if(status != L_SUCCESS) { M_FAIL("GetParameter",status); goto end; } else M_OK("GetParameter",std::endl);
   std::cout << ".......... Point size:                   " << pointsize << std::endl << std::endl;

   status = pI->InitStartLDevice(); // Инициализируем внутренние переменные драйвера
   if(status != L_SUCCESS) { M_FAIL("InitStartLDevice(ADC)",status); goto end; } else M_OK("InitStartLDevice(ADC)",std::endl);

   //hThread=CreateThread(0,0x2000,ServiceThread,0,0,&Tid); // Создаем и запускаем поток сбора данных
  
   status=pI->StartLDevice(); // Запускаем сбор в драйвере
   if(status != L_SUCCESS) { M_FAIL("StartLDevice(ADC)",status); goto end; } else M_OK("StartLDevice(ADC)",std::endl);

   //th.join();

    ULONG halfbuffer = IrqStep*pages/2;              // Собираем половинками кольцевого буфера
    std::atomic<ULONG> s;
    ULONG f1, f2;
       
    s.exchange(*sync);
    f1 = (s < halfbuffer) ? 0 : 1;
    f2 = (s < halfbuffer) ? 0 : 1;

    const size_t len{ halfbuffer };

    size_t q{ 0 };
    size_t k{ 0 };
    while (q < 4) {
        while (f1 == f2) {
            s.exchange(*sync);
            f2 = (s < halfbuffer) ? 0 : 1;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        s.exchange(*sync);
        f1 = (s < halfbuffer) ? 0 : 1;

        int16_t* tmp = (int16_t*)data + halfbuffer * ((f1 == 1) ? 0 : 1);

        for (size_t j = 0; (j + 1) < len; j += 2, k++) {
            std::cout << k << ";" << tmp[j] << ";" << tmp[j + 1] << std::endl;
        }
        for (size_t j = 0; j < 5; j++, k++) {
            std::cout << k << ";" << 100000 << ";" << 100000 << std::endl;
        }

        //auto d1 = calc_rms(tmp, len / 2, 0, 2);
        //auto d2 = calc_rms(tmp, len / 2, 1, 2);
        //std::cout << d1 << " " << d2 << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        q++;
    }

   status = pI->StopLDevice(); // Остановили сбор
   if(status != L_SUCCESS) { M_FAIL("StopLDevice(ADC)",status); goto end; } else M_OK("StopLDevice(ADC)",std::endl);

end:
    status = pI->CloseLDevice();
    if(status != L_SUCCESS) { M_FAIL("CloseLDevice",status); /*goto end;*//* } else M_OK("CloseLDevice",std::endl);

    status = pI->Release();
    M_OK("Release IDaqLDevice",std::endl);
    std::cout << ".......... Ref: " << status << std::endl;

    FreeLibrary(lib);
}*/
