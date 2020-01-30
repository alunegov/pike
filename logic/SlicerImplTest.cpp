#include <catch2/catch.hpp>
#include <catch/fakeit.hpp>

#include <SlicerImpl.h>

TEST_CASE("SlicerImpl", "[SlicerImpl]") {
    fakeit::Mock<ros::devices::Ender> ender1_mock;
    auto& ender1 = ender1_mock.get();

    fakeit::Mock<ros::devices::Ender> ender2_mock;
    auto& ender2 = ender2_mock.get();

    fakeit::Mock<ros::devices::Rotator> rotator_mock;
    auto& rotator = rotator_mock.get();

    fakeit::Mock<ros::devices::Inclinometer> inclinometer_mock;
    auto& inclinometer = inclinometer_mock.get();

    fakeit::Mock<ros::devices::Depthometer> depthometer_mock;
    auto& depthometer = depthometer_mock.get();

    fakeit::Mock<ros::pike::logic::Pike> pike_mock;
    fakeit::When(Method(pike_mock, ender1)).AlwaysReturn(&ender1);
    fakeit::When(Method(pike_mock, ender2)).AlwaysReturn(&ender2);
    fakeit::When(Method(pike_mock, rotator)).AlwaysReturn(&rotator);
    fakeit::When(Method(pike_mock, inclinometer)).AlwaysReturn(&inclinometer);
    fakeit::When(Method(pike_mock, depthometer)).AlwaysReturn(&depthometer);
    auto& pike = pike_mock.get();

    ros::pike::logic::SlicerImpl sut{&pike};

    SECTION("Read") {
        fakeit::When(Method(ender1_mock, Get)).Return(false, true, true, false, false, false);

        fakeit::When(Method(ender2_mock, Get)).Return(false, false, false, false, true, true);

        fakeit::When(Method(rotator_mock, StepsIn360)).AlwaysReturn(360);
        fakeit::Fake(Method(rotator_mock, Enable));
        fakeit::Fake(Method(rotator_mock, SetDirection));
        fakeit::Fake(Method(rotator_mock, SetSpeed));
        fakeit::Fake(Method(rotator_mock, PreStep));
        fakeit::Fake(Method(rotator_mock, Step));

        fakeit::When(Method(inclinometer_mock, Get)).Return(10);

        fakeit::When(Method(depthometer_mock, Read)).Return(11, 12, 13);

        fakeit::Fake(Method(pike_mock, ReadAndUpdateTtlIn));

        std::atomic_bool cancel_token{false};

        fakeit::Mock<ros::pike::logic::SlicerReadOutput> output_mock;
        fakeit::Fake(Method(output_mock, SliceTick));
        fakeit::Fake(Method(output_mock, TtlInTick));
        auto& output = output_mock.get();
        
        auto slice_msr = sut.Read(cancel_token, &output);

        fakeit::Verify(Method(rotator_mock, StepsIn360)).AtLeastOnce();
        fakeit::Verify(Method(rotator_mock, Enable)).Once();
        fakeit::Verify(Method(rotator_mock, SetDirection)).Exactly(2);
        fakeit::Verify(Method(rotator_mock, SetSpeed)).Exactly(2);
        fakeit::Verify(Method(rotator_mock, PreStep)).Exactly(2);
        fakeit::Verify(Method(rotator_mock, Step)).Exactly(4);

        fakeit::Verify(Method(pike_mock, ReadAndUpdateTtlIn)).Exactly(6);

        fakeit::Verify(Method(output_mock, SliceTick)).Exactly(3);
        fakeit::Verify(Method(output_mock, TtlInTick)).Exactly(6);

        REQUIRE(slice_msr.ok);
        REQUIRE(!slice_msr.ec);
        REQUIRE(slice_msr.inclio_angle == 10);
        REQUIRE(slice_msr.angles == std::vector<double>{0, 1, 2});
        REQUIRE(slice_msr.depths == std::vector<int16_t>{11, 12, 13});
    }
}
