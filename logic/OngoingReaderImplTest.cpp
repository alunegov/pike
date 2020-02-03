#include <chrono>
#include <thread>

#include <catch2/catch.hpp>

#if !defined(CATCH_INTERNAL_UNSUPPRESS_PARENTHESES_WARNINGS)
#   define CATCH_INTERNAL_UNSUPPRESS_PARENTHESES_WARNINGS
#endif
#include <catch/fakeit.hpp>

#include <OngoingReaderImpl.h>

TEST_CASE("OngoingReaderImpl", "[OngoingReaderImpl]") {
    fakeit::Mock<ros::dc::DAQ> daq_mock;
    auto& daq = daq_mock.get();

    fakeit::Mock<ros::devices::Ender> ender1_mock;
    auto& ender1 = ender1_mock.get();

    fakeit::Mock<ros::devices::Ender> ender2_mock;
    auto& ender2 = ender2_mock.get();

    fakeit::Mock<ros::devices::Odometer> odometer_mock;
    auto& odometer = odometer_mock.get();

    fakeit::Mock<ros::devices::Inclinometer> inclinometer_mock;
    auto& inclinometer = inclinometer_mock.get();

    fakeit::Mock<ros::devices::Depthometer> depthometer_mock;
    auto& depthometer = depthometer_mock.get();

    fakeit::Mock<ros::pike::logic::Pike> pike_mock;
    fakeit::When(Method(pike_mock, daq)).AlwaysReturn(&daq);
    fakeit::When(Method(pike_mock, ender1)).AlwaysReturn(&ender1);
    fakeit::When(Method(pike_mock, ender2)).AlwaysReturn(&ender2);
    fakeit::When(Method(pike_mock, odometer)).AlwaysReturn(&odometer);
    fakeit::When(Method(pike_mock, inclinometer)).AlwaysReturn(&inclinometer);
    fakeit::When(Method(pike_mock, depthometer)).AlwaysReturn(&depthometer);
    fakeit::Fake(Method(pike_mock, ReadAndUpdateTtlIn));
    auto& pike = pike_mock.get();

    ros::pike::logic::OngoingReaderImpl sut{&pike, 1};

    using AdcRead2Func = tl::expected<void, std::error_code>(double_t&, const ros::dc::DAQ::_Channels&,
            const std::atomic_bool&, const std::function<ros::dc::DAQ::AdcReadCallback>&);

    SECTION("Start/Stop and adc thread") {
        const auto adcRead2 = [](double_t& reg_freq, const ros::dc::DAQ::_Channels& channels,
                const std::atomic_bool& cancel_token, const std::function<ros::dc::DAQ::AdcReadCallback>& callback) -> tl::expected<void, std::error_code> {
            callback(nullptr, 0);
            callback(nullptr, 0);
            return {};
        };

        fakeit::When(OverloadedMethod(daq_mock, AdcRead, AdcRead2Func)).Do(adcRead2);

        fakeit::Fake(Method(odometer_mock, FillChannels));
        fakeit::Fake(Method(odometer_mock, Update));
        fakeit::When(Method(odometer_mock, Get)).Return(1, 2);

        fakeit::Fake(Method(inclinometer_mock, FillChannels));
        fakeit::Fake(Method(inclinometer_mock, Update));
        fakeit::When(Method(inclinometer_mock, Get)).Return(11, 12);

        fakeit::Mock<ros::pike::logic::OngoingReaderOutput> output_mock;
        fakeit::Fake(Method(output_mock, AdcTick));
        fakeit::Fake(Method(output_mock, AdcTick_Values));
        auto& output = output_mock.get();

        sut.SetOutput(&output);
        sut.IdleDepth(true);

        sut.Start();
        // в потоке adc будет вызвана adcRead2, Stop заблокируется до завершения потоков
        sut.Stop();

        fakeit::Verify(Method(odometer_mock, FillChannels)).Once();
        fakeit::Verify(Method(odometer_mock, Update)).Exactly(2);

        fakeit::Verify(Method(inclinometer_mock, FillChannels)).Once();
        fakeit::Verify(Method(inclinometer_mock, Update)).Exactly(2);

        fakeit::Verify(Method(output_mock, AdcTick).Using(1, 11));
        fakeit::Verify(Method(output_mock, AdcTick).Using(2, 12));
        fakeit::VerifyNoOtherInvocations(Method(output_mock, AdcTick));
        fakeit::Verify(Method(output_mock, AdcTick_Values)).Exactly(2);
    }

    SECTION("Start/Stop and misc thread") {
        fakeit::Fake(OverloadedMethod(daq_mock, AdcRead, AdcRead2Func));

        fakeit::When(Method(ender1_mock, Get)).AlwaysReturn(false);

        fakeit::When(Method(ender2_mock, Get)).AlwaysReturn(true);

        fakeit::Fake(Method(odometer_mock, FillChannels));

        fakeit::Fake(Method(inclinometer_mock, FillChannels));

        fakeit::When(Method(depthometer_mock, Read)).AlwaysReturn(10);

        fakeit::Mock<ros::pike::logic::OngoingReaderOutput> output_mock;
        fakeit::Fake(Method(output_mock, DepthTick));
        fakeit::Fake(Method(output_mock, TtlInTick));
        auto& output = output_mock.get();

        sut.SetOutput(&output);
        sut.IdleDepth(false);

        sut.Start();
        // поток adc сразу завершится, Stop заблокируется до завершения потоков
        std::this_thread::sleep_for(std::chrono::milliseconds{777});
        sut.Stop();

        fakeit::Verify(Method(output_mock, DepthTick).Using(10));
        fakeit::VerifyNoOtherInvocations(Method(output_mock, DepthTick));
        fakeit::Verify(Method(output_mock, TtlInTick).Using(false, true));
        fakeit::VerifyNoOtherInvocations(Method(output_mock, TtlInTick));
    }
}