#include <catch2/catch.hpp>

#include <ConfMapper.h>

TEST_CASE("ConfMapper Load", "[ConfMapper]") {
    const auto res = ros::pike::logic::ConfMapper::Load("ref_conf.json");

    REQUIRE(res.object_length == 110);

    REQUIRE(res.daq.slot == 0);
    REQUIRE(res.daq.common_gnd);
    REQUIRE(res.daq.adc_rate == 12.8);

    REQUIRE(res.depthometer.port_name == "\\\\.\\COM43");
    REQUIRE(res.depthometer.baud_rate == 115200);

    REQUIRE(res.ender1.pin == 3);

    REQUIRE(res.ender2.pin == 4);

    REQUIRE(res.inclinometer.x_channel == 1);
    REQUIRE(res.inclinometer.y_channel == 2);
    REQUIRE(res.inclinometer.trans_table_file == "inclinometer.tbl");

    REQUIRE(res.mover.pwm_pin == 1);
    REQUIRE(res.mover.dir_pin == 2);

    REQUIRE(res.odometer.a_channel == 5);
    REQUIRE(res.odometer.b_channel == 6);
    REQUIRE(res.odometer.threshold == 2.0);
    REQUIRE(res.odometer.distance_per_pulse == 0.36);

    REQUIRE(res.rotator.en_pin == 3);
    REQUIRE(res.rotator.step_pin == 5);
    REQUIRE(res.rotator.dir_pin == 4);
    REQUIRE(res.rotator.mx_pin == 6);
    REQUIRE(res.rotator.steps_per_msr == 24686);
    REQUIRE(res.rotator.steps_per_view == 1543);

    REQUIRE(res.remote.port == 45454);
}
