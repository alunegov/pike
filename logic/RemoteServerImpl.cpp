#include <RemoteServerImpl.h>

#include <cassert>
#include <chrono>
#include <cmath>

namespace ros { namespace pike { namespace logic {

RemoteServerImpl::~RemoteServerImpl()
{
    _cancel_token = true;
    if (_recv_thread.joinable()) {
        _recv_thread.join();
    }
    if (_reset_thread.joinable()) {
        _reset_thread.join();
    }
}

void RemoteServerImpl::SetOutput(RemoteServerOutput* output)
{
    assert(output != nullptr);
    _output = output;
}

void RemoteServerImpl::Start()
{
    assert(!_recv_thread.joinable() && !_reset_thread.joinable());

    _cancel_token = false;

    _recv_thread = std::thread{[this]() {
        constexpr std::chrono::milliseconds RecvDelay{777};

        while (!_cancel_token) {
            std::this_thread::sleep_for(RecvDelay);

            {
                std::unique_lock<std::mutex> lock{_state_locker};

                // TODO: recv from client
                const double_t x_value{0};
                const double_t y_value{0};

                const MotionDirection new_move_state = (abs(y_value) >= 0.3)
                        ? ((y_value >= 0) ? MotionDirection::Inc : MotionDirection::Dec)
                        : MotionDirection::No;
                const MotionDirection new_rot_state = (abs(x_value) >= 0.3)
                        ? ((x_value >= 0) ? MotionDirection::Inc : MotionDirection::Dec)
                        : MotionDirection::No;

                if (_move_state != new_move_state) {
                    if (_rot_state == MotionDirection::No) {  // нет вращения
                        if (_move_state != MotionDirection::No) {
                            _output->RemoteStopMovement();
                        }
                        if (new_move_state != MotionDirection::No) {
                            _output->RemoteStartMovement(new_move_state);
                        }
                    }
                    _move_state = new_move_state;
                }

                if (_rot_state != new_rot_state) {
                    if (_move_state == MotionDirection::No) {  // нет движения
                        if (_rot_state != MotionDirection::No) {
                            _output->RemoteStopRotation();
                        }
                        if (new_rot_state != MotionDirection::No) {
                            _output->RemoteStartRotation(new_rot_state);
                        }
                    }
                    _rot_state = new_rot_state;
                }

                _last_state = std::chrono::steady_clock::now();
            }
        }
    }};

    _reset_thread = std::thread{[this]() {
        // Задержка между проверками для авто-сброса движения
        constexpr std::chrono::seconds ResetDelay{1};
        // Период "неприхода" данных от клиента, после которого будет авто-сброс движения
        constexpr std::chrono::seconds ResetDelta{5};

        while (!_cancel_token) {
            std::this_thread::sleep_for(ResetDelay);

            {
                std::unique_lock<std::mutex> lock{_state_locker};

                // сбрасываем движение, если прошло больше ResetDelta
                const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - _last_state);
                if (elapsed > ResetDelta) {
                    if (_move_state != MotionDirection::No) {
                        _output->RemoteStopMovement();
                        _move_state = MotionDirection::No;
                    }
                    if (_rot_state != MotionDirection::No) {
                        _output->RemoteStopRotation();
                        _rot_state = MotionDirection::No;
                    }
                }
            }
        }
    }};
}

void RemoteServerImpl::Stop()
{
    assert(_recv_thread.joinable() && _reset_thread.joinable());

    _cancel_token = true;
    _recv_thread.join();
    _reset_thread.join();
}

}}}
