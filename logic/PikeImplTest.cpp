#include <system_error>

#include <tl/expected.hpp>

#include <catch2/catch.hpp>
#include <catch/fakeit.hpp>

#include <PikeImpl.h>

TEST_CASE("PikeImpl", "[PikeImpl]") {
    fakeit::Mock<ros::dc::DAQ> daq_mock;
    auto& daq = daq_mock.get();

    fakeit::Mock<ros::devices::Ender> ender1_mock;
    auto& ender1 = ender1_mock.get();

    fakeit::Mock<ros::devices::Ender> ender2_mock;
    auto& ender2 = ender2_mock.get();

    ros::pike::logic::PikeImpl sut{&daq, &ender1, &ender2, nullptr, nullptr, nullptr, nullptr, nullptr};

    SECTION("InMotion") {
        REQUIRE(!sut.InMotion());

        sut.SetIsMoving(true);
        REQUIRE(sut.InMotion());

        sut.SetIsMoving(false);
        REQUIRE(!sut.InMotion());

        sut.SetIsRotating(true);
        REQUIRE(sut.InMotion());

        sut.SetIsMoving(true);
        REQUIRE(sut.InMotion());
    }

    SECTION("IsMoving") {
        REQUIRE(!sut.IsMoving());

        sut.SetIsMoving(true);
        REQUIRE(sut.IsMoving());

        sut.SetIsMoving(false);
        REQUIRE(!sut.IsMoving());
    }

    SECTION("IsRotating") {
        REQUIRE(!sut.IsRotating());

        sut.SetIsRotating(true);
        REQUIRE(sut.IsRotating());

        sut.SetIsRotating(false);
        REQUIRE(!sut.IsRotating());
    }

    SECTION("IsIsSlicing") {
        REQUIRE(!sut.IsSlicing());

        sut.SetIsSlicing(true);
        REQUIRE(sut.IsSlicing());

        sut.SetIsSlicing(false);
        REQUIRE(!sut.IsSlicing());
    }

    SECTION("ReadAndUpdateTtlIn should update enders on daq.TtlIn success") {
        fakeit::When(Method(daq_mock, TtlIn)).Return(3);

        fakeit::Fake(Method(ender1_mock, Update).Using(3));

        fakeit::Fake(Method(ender2_mock, Update).Using(3));

        const auto opt = sut.ReadAndUpdateTtlIn();

        REQUIRE(opt);
    }

    SECTION("ReadAndUpdateTtlIn shouldn`t update enders on daq.TtlIn failure") {
        fakeit::When(Method(daq_mock, TtlIn)).Return(tl::make_unexpected(std::make_error_code(std::errc::bad_address)));

        const auto opt = sut.ReadAndUpdateTtlIn();

        REQUIRE(!opt);
    }
}
