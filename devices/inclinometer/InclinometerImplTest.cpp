#include <catch2/catch.hpp>

#include <InclinometerImpl.h>

const uint16_t x_channel{0};
const uint16_t y_channel{1};
const double_t adc_to_volt{1};

TEST_CASE("InclinometerImpl usual workflow", "[InclinometerImpl]") {
    std::vector<ros::devices::InclinometerImpl::_Entry> trans_table{
        {1, 4.1, 1.1},
        {0, 2.58, 2.525},
        {-1, 1.06, 4.15},
    };

    ros::devices::InclinometerImpl sut{x_channel, y_channel, trans_table};

    REQUIRE(sut.Get() == 0);

    SECTION("FillChannels should fill channels with x_channel and y_channel") {
        std::vector<uint16_t> channels;

        sut.FillChannels(channels);

        REQUIRE(channels == std::vector<uint16_t>{x_channel, y_channel});

        SECTION("Update should recalc inclio") {
            const std::vector<int16_t> values1{0, 0, 3, 3, 0, 0, 3, 3, 0, 0, 3, 3, 0, 0, 3, 3};
            sut.Update(channels, values1.data(), values1.size(), adc_to_volt);

            REQUIRE(sut.Get() != 0);
        }
    }
}
