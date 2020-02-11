#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <thread>

#include <QtNetwork/QUdpSocket>

#include <RemoteServer.h>

namespace ros { namespace pike { namespace logic {

class RemoteServerImpl : public RemoteServer
{
public:
    explicit RemoteServerImpl(uint16_t port) :
        _port{port}
    {}

    ~RemoteServerImpl() override;

    // RemoteServer

    void SetOutput(RemoteServerOutput* output) override;

    void Start() override;

    void Stop() override;

private:
    // Задержка между проверками для авто-сброса движения
    const std::chrono::seconds ResetDelay{1};
    // Период "неприхода" данных от клиента, после которого будет авто-сброс движения
    const std::chrono::seconds ResetDelta{5};

    void NonVirtualStop();

    void ReadPendingDatagrams();

    void ProcessMotionData(double_t x_value, double_t y_value);

    void AutoResetMotion();

    uint16_t _port{0};

    RemoteServerOutput* _output{nullptr};

    QUdpSocket* _recv_socket{nullptr};

    std::thread _auto_reset_thread;
    std::atomic_bool _cancel_token{false};

    std::mutex _state_locker;
    MotionDirection _move_state{MotionDirection::No};
    MotionDirection _rot_state{MotionDirection::No};
    std::chrono::steady_clock::time_point _last_state{std::chrono::steady_clock::now()};
};

}}}
