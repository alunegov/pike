#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include <RemoteServer.h>

namespace ros { namespace pike { namespace logic {

class RemoteServerImpl : public RemoteServer
{
public:
    ~RemoteServerImpl() override;

    // RemoteServer

    void SetOutput(RemoteServerOutput* output) override;

    void Start() override;

    void Stop() override;

private:
    RemoteServerOutput* _output{nullptr};

    std::thread _recv_thread;
    std::thread _reset_thread;
    std::atomic_bool _cancel_token{false};

    std::mutex _state_locker;
    MotionDirection _move_state{MotionDirection::No};
    MotionDirection _rot_state{MotionDirection::No};
    std::chrono::steady_clock::time_point _last_state{std::chrono::steady_clock::now()};
};

}}}
