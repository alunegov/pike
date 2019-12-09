#pragma once

#include <cmath>
#include <cstdint>
#include <vector>
#include <utility>

#define INITGUID
#include <Windows.h>

#include <ioctl.h>
#include <ifc_ldev.h>

namespace ros { namespace dc { namespace lcard {

class LCardDevice {
public:
    LCardDevice() = default;
    virtual ~LCardDevice();

    void Init();
    void Deinit();

    void TtlEnable(bool enable);
    void TtlOut(uint16_t value);
    uint16_t TtlIn();

    void AdcRead(double_t& reg_freq, size_t point_count, const std::vector<uint16_t>& channels, int16_t* values);

private:
    // копия функции ___GetRate из проекта UsbE_dll_v2
    std::pair<uint16_t, uint16_t> GetRate(double_t channelRate, size_t channelCount, double_t eps);

    HINSTANCE lcomp_handle_{ 0 };
    IDaqLDevice* device_{ nullptr };
    ULONG board_type_{ NONE };
};

}}}
