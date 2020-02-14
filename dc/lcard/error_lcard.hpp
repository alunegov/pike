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
    PlataTestErr,
    ReadPlataDescrErr,
    EnableCorrectionErr,
    DetectAdcRateParamsErr,
    IoAsyncErr,
    InitStartLDeviceErr,
    StartLDeviceErr,
    StopLDeviceErr,
    RequestBufferStreamErr,
    FillDAQparametersErr,
    SetParametersStreamErr,
    GetParameterErr,
    SetRegFreqErr,
    NotFullCadrInHalfBuffer,
};

}

REGISTER_ERROR_CODES(ros, error_lcard);
