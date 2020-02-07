#include <catch2/catch.hpp>

#include <ConfMapper.h>

TEST_CASE("ConfMapper Load", "[ConfMapper]") {
    const auto res = ros::pike::logic::ConfMapper::Load("ref_conf.json");

    CHECK(res.object_length == 110);

    CHECK(res.daq.slot == 0);
    CHECK(res.daq.common_gnd);
    CHECK(res.daq.adc_rate == 12.8);

    CHECK(res.depthometer.port_name == "\\\\.\\COM43");
    CHECK(res.depthometer.baud_rate == 115200);

    CHECK(res.ender1.pin == 3);

    CHECK(res.ender2.pin == 4);

    CHECK(res.inclinometer.x_channel == 1);
    CHECK(res.inclinometer.y_channel == 2);
    CHECK(res.inclinometer.trans_table_file == "inclinometer.tbl");

    CHECK(res.mover.pwm_pin == 1);
    CHECK(res.mover.dir_pin == 2);

    CHECK(res.odometer.a_channel == 5);
    CHECK(res.odometer.b_channel == 6);
    CHECK(res.odometer.threshold == 2.0);
    CHECK(res.odometer.distance_per_pulse == 0.36);

    CHECK(res.rotator.en_pin == 3);
    CHECK(res.rotator.step_pin == 5);
    CHECK(res.rotator.dir_pin == 4);
    CHECK(res.rotator.mx_pin == 6);
    CHECK(res.rotator.steps_per_msr == 24686);
    CHECK(res.rotator.steps_per_view == 1543);

    CHECK(res.remote.port == 45454);
}
