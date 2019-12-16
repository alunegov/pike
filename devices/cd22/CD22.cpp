#include <CD22.h>

#include <chrono>
//#include <iomanip>
//#include <iostream>

namespace ros { namespace devices {

CD22::~CD22()
{
    transport_.Close();
}

int16_t CD22::Read()
{
    const int16_t NoValue{INT16_MIN};

    uint8_t query[] = {0x02, 0x43, 0xb0, 0x01, 0x03, 0xf2};
    uint8_t ans[6];
    bool r;

    if (!transport_.Write(reinterpret_cast<char*>(query), 6)) {
        //std::cout << "Port write error" << std::endl;
        return NoValue;
    }

    memset(ans, 0, sizeof(ans));
    for (size_t i = 0; i < 6; i++) {
        auto c = transport_.ReadChar(r);
        if (r) {
            //std::cout << "Char at " << i << " is " << c << std::endl;
            ans[i] = c;
        } else {
            //std::cout << "Port read error at " << i << " byte" << std::endl;
            return NoValue;
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
        return NoValue;
    }
    // error check
    if (ans[1] == 0x15) {
        //std::cout << "error code " << std::hex << static_cast<int>(ans[2]) << " " << std::endl;
        return NoValue;
    }
    // ACK check
    if (ans[1] != 0x06) {
        //std::cout << "not an ACK but " << std::hex << static_cast<int>(ans[1]) << " " << std::endl;
        return NoValue;
    }

    return static_cast<int16_t>((ans[2] << 8) | ans[3]);
}

}}
