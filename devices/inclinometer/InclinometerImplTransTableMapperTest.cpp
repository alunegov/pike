#include <fstream>
#include <string>

#include <catch2/catch.hpp>

#include <InclinometerImplTransTableMapper.h>

TEST_CASE("InclinometerImplTransTableMapper Load", "[InclinometerImplTransTableMapper]") {
    const std::string filename{"test_inclinometer.tbl"};

    {
        std::ofstream file{filename};

        file << 1 << " " << 4.1 << " " << 1.1 << "\n"
                << 0 << " " << 2.58 << " " << 2.525 << "\n"
                << -1 << " " << 1.06 << " " << 4.15 << "\n";
    }

    const auto res = ros::devices::InclinometerImplTransTableMapper::Load(filename);

    REQUIRE(res.size() == 3);
    REQUIRE(res[0] == ros::devices::InclinometerImplTransTableEntry{1, 4.1, 1.1});
    REQUIRE(res[1] == ros::devices::InclinometerImplTransTableEntry{0, 2.58, 2.525});
    REQUIRE(res[2] == ros::devices::InclinometerImplTransTableEntry{-1, 1.06, 4.15});
}
