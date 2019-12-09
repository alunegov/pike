#include <CD22.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

#include <ceSerial.h>

namespace ros { namespace devices {

int16_t CD22::ReadDist()
{
    uint8_t query[] = {0x02, 0x43, 0xb0, 0x01, 0x03, 0xf2};
    uint8_t ans[6];
    bool r;

    if (!transport_.Write(reinterpret_cast<char*>(query), 6)) {
        //std::cout << "Port write error" << std::endl;
        return 0;
    }

    memset(ans, 0, sizeof(ans));
    for (size_t i = 0; i < 6; i++) {
        auto c = transport_.ReadChar(r);
        if (r) {
            //std::cout << "Char at " << i << " is " << c << std::endl;
            ans[i] = c;
        } else {
            //std::cout << "Port read error at " << i << " byte" << std::endl;
            return 0;
        }
    }

    /*std::cout << "answer is " << std::hex;
    for (auto a : ans) {
        std::cout << static_cast<int>(a) << " ";
    }
    std::cout << std::endl;*/

    // BCC check
    if ((ans[1] ^ ans[2] ^ ans[3]) != ans[5]) {
        //std::cout << "an BCC is invalid" << std::endl;
        return 0;
    }
    // error check
    if (ans[1] == 0x15) {
        //std::cout << "error code " << std::hex << static_cast<int>(ans[2]) << " " << std::endl;
        return 0;
    }
    // ACK check
    if (ans[1] != 0x06) {
        //std::cout << "not an ACK but " << std::hex << static_cast<int>(ans[1]) << " " << std::endl;
        return 0;
    }

    return static_cast<int16_t>((ans[2] << 8) | ans[3]);
}

}}
