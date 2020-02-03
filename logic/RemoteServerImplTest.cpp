#include <QtCore/qtestsupport_core.h>

#include <QtCore/QCoreApplication>
#include <QtCore/QDataStream>
#include <QtNetwork/QUdpSocket>

#include <catch2/catch.hpp>

#if !defined(CATCH_INTERNAL_UNSUPPRESS_PARENTHESES_WARNINGS)
#   define CATCH_INTERNAL_UNSUPPRESS_PARENTHESES_WARNINGS
#endif
#include <catch/fakeit.hpp>

#include <RemoteServerImpl.h>

TEST_CASE("RemoteServerImpl", "[RemoteServerImpl]") {
    // for event-loop startup
    int argc{0};
    QCoreApplication app{argc, nullptr};

    const uint16_t Port{45454};

    ros::pike::logic::RemoteServerImpl sut{Port};

    fakeit::Mock<ros::pike::logic::RemoteServerOutput> output_mock;
    fakeit::Fake(Method(output_mock, RemoteStartMovement));
    fakeit::Fake(Method(output_mock, RemoteStopMovement));
    fakeit::Fake(Method(output_mock, RemoteStartRotation));
    fakeit::Fake(Method(output_mock, RemoteStopRotation));
    auto& output = output_mock.get();

    sut.SetOutput(&output);

    sut.Start();

    QUdpSocket client;

    const auto send = [&client, Port](double_t x, double_t y) {
        QByteArray datagram;
        QDataStream ds{&datagram, QIODevice::WriteOnly};
        ds << x << y;
        client.writeDatagram(datagram, QHostAddress::Broadcast, Port);
    };

    // fwd below threshold - skiped
    send(0.0, 0.2);
    // fwd
    send(0.0, 1.0);
    // back below threshold - move stop
    send(0.0, -0.2);
    // back
    send(0.0, -1.0);
    // fwd
    send(0.0, 1.0);
    // cw while move - skiped
    send(1.0, 1.0);
    // cw below threshold - move stop
    send(0.2, 0.0);
    // cw
    send(1.0, 0.0);
    // ccw below threshold - rotation stop
    send(-0.2, 0.0);
    // ccw
    send(-1.0, 0.0);
    // cw
    send(1.0, 0.0);
    // fwd while rotation - skiped
    send(1.0, 1.0);
    // full stop (rotation) via auto-reset 

    // 5 + 1 сек - время авто-сброса движения
    QTest::qWaitFor([]() { return false; }, 6100);

    sut.Stop();

    fakeit::Verify(
        Method(output_mock, RemoteStartMovement).Using(ros::pike::logic::MotionDirection::Inc)
        + Method(output_mock, RemoteStopMovement)
        + Method(output_mock, RemoteStartMovement).Using(ros::pike::logic::MotionDirection::Dec)
        + Method(output_mock, RemoteStopMovement)
        + Method(output_mock, RemoteStartMovement).Using(ros::pike::logic::MotionDirection::Inc)
        + Method(output_mock, RemoteStopMovement)
        + Method(output_mock, RemoteStartRotation).Using(ros::pike::logic::MotionDirection::Inc)
        + Method(output_mock, RemoteStopRotation)
        + Method(output_mock, RemoteStartRotation).Using(ros::pike::logic::MotionDirection::Dec)
        + Method(output_mock, RemoteStopRotation)
        + Method(output_mock, RemoteStartRotation).Using(ros::pike::logic::MotionDirection::Inc)
        + Method(output_mock, RemoteStopRotation)
    );
    fakeit::VerifyNoOtherInvocations(output_mock);
}
