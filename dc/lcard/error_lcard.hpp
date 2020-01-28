#pragma once

#include <errors.hpp>

namespace ros {

enum class error_lcard
{
    generic = 1,
    LoadLibraryErr,
    FreeLibraryErr,
    CreateInstanceAddrErr,
    CreateInstanceErr,
    QueryInterfaceErr,
    OpenLDeviceErr,
    CloseLDeviceErr,
    GetSlotParamErr,
    LoadBiosErr,
    ReadPlataDescrErr,
    DetectAdcRateParamsErr,
    IoAsyncErr,
    PrepareAdcErr,
    InitStartLDeviceErr,
    StartLDeviceErr,
    StopLDeviceErr,
};

}

REGISTER_ERROR_CODES(ros, error_lcard);