#include <cmath>
#include <cstdint>

#include <catch2/catch.hpp>

#include <OdometerImpl.h>

const uint16_t a_channel{0};
const uint16_t b_channel{1};
const double_t adc_to_volt{1};
const double_t threshold{2};
const double_t distance_per_pulse{1};

TEST_CASE("odometer usual workflow", "[odometer]") {
    ros::devices::OdometerImpl sut{a_channel, b_channel, threshold, distance_per_pulse};

    REQUIRE(sut.Get() == 0);

    SECTION("FillChannels should fill channels with a_channel and b_channel") {
        std::vector<uint16_t> channels;

        sut.FillChannels(channels);

        REQUIRE(channels.size() == 2);
        REQUIRE(channels[0] == a_channel);
        REQUIRE(channels[1] == b_channel);

        SECTION("Update should recalc distance") {
            //     _   _   _   _
            // A _| |_| |_| |_| 
            //     _   _   _   _
            // B _| |_| |_| |_| 
            std::vector<int16_t> values1{0, 0, 3, 3, 0, 0, 3, 3, 0, 0, 3, 3, 0, 0, 3, 3};
            sut.Update(channels, values1.data(), values1.size(), adc_to_volt);

            REQUIRE(sut.Get() == 4 * distance_per_pulse);

            SECTION("repated Update should consider previous state") {
                //   _   _   _
                // A  |_| |_| 
                //     _   _  
                // B _| |_| |_
                std::vector<int16_t> values2{3, 0, 0, 3, 3, 0, 0, 3, 3, 0};
                sut.Update(channels, values2.data(), values2.size(), adc_to_volt);

                REQUIRE(sut.Get() == 2 * distance_per_pulse);

                SECTION("Reset should zero distance") {
                    sut.Reset();

                    REQUIRE(sut.Get() == 0);
                }
            }
        }
    }
}

TEST_CASE("odometer ignore irrelevant adc channels", "[odometer]") {
    ros::devices::OdometerImpl sut{a_channel, b_channel, threshold, distance_per_pulse};

    REQUIRE(sut.Get() == 0);

    SECTION("Update should recalc distance") {
        std::vector<uint16_t> channels{3, a_channel, b_channel};

        //     _   _   _   _
        // A _| |_| |_| |_| 
        //     _   _   _   _
        // B _| |_| |_| |_| 
        std::vector<int16_t> values{1, 0, 0, 1, 3, 3, 1, 0, 0, 1, 3, 3, 1, 0, 0, 1, 3, 3, 1, 0, 0, 1, 3, 3};
        sut.Update(channels, values.data(), values.size(), adc_to_volt);

        REQUIRE(sut.Get() == 4 * distance_per_pulse);
    }
}

TEST_CASE("odometer account adc_to_volt coeff", "[odometer]") {
    ros::devices::OdometerImpl sut{a_channel, b_channel, threshold, distance_per_pulse};

    REQUIRE(sut.Get() == 0);

    SECTION("Update should recalc distance") {
        std::vector<uint16_t> channels{a_channel, b_channel};

        //     _
        // A _| |_ _ _ _ _
        //     _
        // B _| |_ _ _ _ _
        std::vector<int16_t> values{0, 0, 30, 30, 0, 0, 3, 3, 0, 0, 3, 3, 0, 0, 3, 3};
        sut.Update(channels, values.data(), values.size(), 0.1);

        REQUIRE(sut.Get() == 1 * distance_per_pulse);
    }
}
