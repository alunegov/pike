#include <OngoingReaderImpl.h>

#include <cassert>
#include <chrono>
#include <cstdint>
#include <future>
#include <vector>

namespace ros { namespace pike { namespace logic {

OngoingReaderImpl::~OngoingReaderImpl()
{
    _cancel_token = true;
    if (_adc_gather_thread.joinable()) {
        _adc_gather_thread.join();
    }
    /*if (_adc_process_thread.joinable()) {
        _adc_process_thread.join();
    }*/
    if (_misc_thread.joinable()) {
        _misc_thread.join();
    }
}

void OngoingReaderImpl::SetOutput(OngoingReaderOutput* output)
{
    assert(!_adc_gather_thread.joinable()/* && !_adc_process_thread.joinable()*/ && !_misc_thread.joinable());  // ����� �������� ������ � ����������� ���������
    assert(output != nullptr);
    _output = output;
}

void OngoingReaderImpl::Start()
{
    assert(_output != nullptr);
    assert(!_adc_gather_thread.joinable()/* && !_adc_process_thread.joinable()*/ && !_misc_thread.joinable());

    _cancel_token = false;

    _adc_gather_thread = std::thread{[this]() {
        const double_t AdcToVolt{10.0 / 8000};  // TODO: get AdcToVolt from daq

        double_t regFreq{_adc_rate};

        std::vector<uint16_t> channels;
        _pike->inclinometer()->FillChannels(channels);
        _pike->odometer()->FillChannels(channels);

        const auto adc_read_callback = [this, AdcToVolt, &channels](const int16_t* values, size_t values_count) {
            // TODO: ����������� ������ � ��������� ����� � ���������� �� ����

            // spawn 2 parallel threads for inclio and distance
            auto inclio_f = std::async(std::launch::async, [this, AdcToVolt, &channels, values, values_count]() -> double_t {
                _pike->inclinometer()->Update(channels, values, values_count, AdcToVolt);
                return _pike->inclinometer()->Get();
            });
            auto distance_f = std::async(std::launch::async, [this, AdcToVolt, &channels, values, values_count]() -> double_t {
                _pike->odometer()->Update(channels, values, values_count, AdcToVolt);
                return _pike->odometer()->Get();
            });

            const double_t angle = inclio_f.get();
            const double_t distance = distance_f.get();

            _output->AdcTick(distance, angle);
#ifndef NDEBUG
            _output->AdcTick_Values(channels, values, values_count, AdcToVolt);
#endif
        };

        // ������ ������������ ������, �� ����� �������� ����� ���������� adc_read_callback ��� ���������� ��������
        // ������ ���
        // TODO: ����������� ������������� ������ callback (������ �� ������� �� ���� ����� � ���������� ����������� -
        // �������� ���������� �������� ������ ���)
        const auto adc_read_opt = _pike->daq()->AdcRead(regFreq, channels, _cancel_token, adc_read_callback);
        if (!adc_read_opt) {
            // TODO: log and return/output?
        }

        _output->AdcFinish(_cancel_token);
    }};

    /*_adc_process_thread = std::thread{[this]() {
        // TODO: �������� ������� ������ ������ �� ��� (� �������� �� ���������� ��� ������� ������� �����),
        // ���������� �� �� ���������� ������, ���������� ���-��������� � "������" ����� _output

        while (!_cancel_token) {
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        }
    }};*/

    _misc_thread = std::thread{[this]() {
        // �������� ����� �������� depth � TtlIn (��������� ender)
        constexpr std::chrono::milliseconds MiscDelay{333};

        while (!_cancel_token) {
            std::this_thread::sleep_for(MiscDelay);

            if (!_depth_idle_token) {
                const auto depth = _pike->depthometer()->Read();
                if (depth) {
                    _output->DepthTick(depth.value());
                } else {
                    // TODO: log and return/output?
                }

                // ������ ����� ��� ender (�� ���� ������ ttl_in)
                const auto ttlin_opt = _pike->ReadAndUpdateTtlIn();
                if (ttlin_opt) {
                    const bool ender1 = _pike->ender1()->Get();
                    const bool ender2 = _pike->ender2()->Get();

                    _output->TtlInTick(ender1, ender2);
                } else {
                    // TODO: log and return/output?
                }
            }
        }
    }};
}

void OngoingReaderImpl::Stop()
{
    assert(_adc_gather_thread.joinable()/* && _adc_process_thread.joinable()*/ && _misc_thread.joinable());

    _cancel_token = true;
    _adc_gather_thread.join();
    //_adc_process_thread.join();
    _misc_thread.join();
}

void OngoingReaderImpl::IdleDepth(bool value)
{
    _depth_idle_token = value;
}

}}}
