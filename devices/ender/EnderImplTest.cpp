#include <system_error>

#include <tl/expected.hpp>

#include <catch2/catch.hpp>
#include <standalone/fakeit.hpp>

#include <EnderImpl.h>

TEST_CASE("ender Read", "[ender]") {
    const uint16_t Pin{1 - 1};

    fakeit::Mock<ros::dc::DAQ> daq_mock;

    auto& daq = daq_mock.get();
    ros::devices::EnderImpl sut{&daq, Pin};

    SECTION("should update it's state and return state on daq success") {
        REQUIRE(!sut.Get());

        fakeit::When(Method(daq_mock, TtlIn)).Return(1 << Pin);

        auto read_res = sut.Read();

        REQUIRE(read_res);
        REQUIRE(read_res.value());
        REQUIRE(sut.Get());
    }

    SECTION("should not update it's state and return error on daq failure") {
        REQUIRE(!sut.Get());

        fakeit::When(Method(daq_mock, TtlIn)).Return(tl::make_unexpected(std::make_error_code(std::errc::bad_address)));

        auto read_res = sut.Read();

        REQUIRE(!read_res);
        REQUIRE(!sut.Get());
    }
}
