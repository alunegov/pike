#include <RemoteServerImpl.h>

#include <cassert>
#include <chrono>
#include <cmath>

#include <QtCore/QDataStream>
#include <QtNetwork/QNetworkDatagram>

namespace ros { namespace pike { namespace logic {

RemoteServerImpl::~RemoteServerImpl()
{
    delete _recv_socket;

    _cancel_token = true;
    if (_auto_reset_thread.joinable()) {
        _auto_reset_thread.join();
    }
}

void RemoteServerImpl::SetOutput(RemoteServerOutput* output)
{
    assert(_recv_socket == nullptr);
    assert(!_auto_reset_thread.joinable());
    assert(output != nullptr);
    _output = output;
}

void RemoteServerImpl::Start()
{
    assert(_recv_socket == nullptr);
    assert(!_auto_reset_thread.joinable());

    _recv_socket = new QUdpSocket;
    _recv_socket->bind(_port);
    QObject::connect(_recv_socket, &QUdpSocket::readyRead, [this]() { ReadPendingDatagrams(); });

    _cancel_token = false;

    _auto_reset_thread = std::thread{[this]() {
        // Задержка между проверками для авто-сброса движения
        constexpr std::chrono::seconds ResetDelay{1};

        while (!_cancel_token) {
            std::this_thread::sleep_for(ResetDelay);

            AutoResetMotion();
        }
    }};
}

void RemoteServerImpl::Stop()
{
    assert(_recv_socket != nullptr);
    assert(_auto_reset_thread.joinable());

    delete _recv_socket;
    _recv_socket = nullptr;

    _cancel_token = true;
    _auto_reset_thread.join();
}

void RemoteServerImpl::ReadPendingDatagrams()
{
    while (_recv_socket->hasPendingDatagrams()) {
        QNetworkDatagram datagram = _recv_socket->receiveDatagram();

        QDataStream ds{datagram.data()};
        double_t x_value;
        double_t y_value;
        ds >> x_value >> y_value;

        ProcessMotionData(x_value, y_value);
    }
}

void RemoteServerImpl::ProcessMotionData(double_t x_value, double_t y_value)
{
    // Граница детектирования движения, [0-1]
    constexpr double_t MotionThreshold{0.3};

    std::lock_guard<std::mutex> lock{_state_locker};

    const MotionDirection new_move_state = (abs(y_value) >= MotionThreshold)
            ? ((y_value >= 0) ? MotionDirection::Inc : MotionDirection::Dec)
            : MotionDirection::No;
    const MotionDirection new_rot_state = (abs(x_value) >= MotionThreshold)
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

void RemoteServerImpl::AutoResetMotion()
{
    // Период "неприхода" данных от клиента, после которого будет авто-сброс движения
    constexpr std::chrono::seconds ResetDelta{5};

    std::lock_guard<std::mutex> lock{_state_locker};

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

}}}
