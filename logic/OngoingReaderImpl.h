#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <thread>

#include <OngoingReader.h>
#include <Pike.h>

namespace ros { namespace pike { namespace logic {

// Режим измерения пройденного расстояния, положения в пространстве и глубины
class OngoingReaderImpl : public OngoingReader
{
public:
    OngoingReaderImpl() = delete;

    OngoingReaderImpl(ros::pike::logic::Pike* pike, double_t adc_rate) :
        _pike{pike},
        _adc_rate{adc_rate}
    {}

    ~OngoingReaderImpl() override;

    // OngoingReader

    void SetOutput(OngoingReaderOutput* output) override;

    void Start() override;

    void Stop() override;

    void IdleDepth(bool value) override;

private:
    // Периодичность вызова callback из AdcRead
    const std::chrono::milliseconds AdcReadCallbackInterval{555};
    // Задержка между чтениями depth и TtlIn (показания ender)
    const std::chrono::milliseconds MiscDelay{333};

    void NonVirtualStop();

    ros::pike::logic::Pike* _pike{nullptr};

    // Частота регистрации АЦП, кГц
    double_t _adc_rate{0};

    // Выход
    OngoingReaderOutput* _output{nullptr};

    // Поток регистрации АЦП
    std::thread _adc_gather_thread;
    // Поток разбора данных от АЦП (обновление устройств и выдача output)
    //std::thread _adc_process_thread;
    // Поток опроса глубины и TtlIn (показания ender)
    std::thread _misc_thread;
    // Токен останова потоков
    std::atomic_bool _cancel_token{false};

    // Токен приостановки чтения глубины и TtlIn (на время slice)
    std::atomic_bool _depth_idle_token{false};
};

}}}
