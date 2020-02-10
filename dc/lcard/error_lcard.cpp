#include <error_lcard.hpp>

namespace ros {

std::string error_lcard_messages::message(int ev) const
{
    switch (static_cast<error_lcard>(ev)) {
    case error_lcard::LoadLibraryErr:
        return "LoadLibraryErr";
    case error_lcard::FreeLibraryErr:
        return "FreeLibraryErr";
    case error_lcard::CreateInstanceAddrErr:
        return "CreateInstanceAddrErr";
    case error_lcard::CreateInstanceErr:
        return "CreateInstanceErr";
    case error_lcard::QueryInterfaceErr:
        return "QueryInterfaceErr";
    case error_lcard::OpenLDeviceErr:
        return "OpenLDeviceErr";
    case error_lcard::CloseLDeviceErr:
        return "CloseLDeviceErr";
    case error_lcard::GetSlotParamErr:
        return "GetSlotParamErr";
    case error_lcard::LoadBiosErr:
        return "LoadBiosErr";
    case error_lcard::ReadPlataDescrErr:
        return "ReadPlataDescrErr";
    case error_lcard::DetectAdcRateParamsErr:
        return "DetectAdcRateParamsErr";
    case error_lcard::IoAsyncErr:
        return "IoAsyncErr";
    case error_lcard::InitStartLDeviceErr:
        return "InitStartLDeviceErr";
    case error_lcard::StartLDeviceErr:
        return "StartLDeviceErr";
    case error_lcard::StopLDeviceErr:
        return "StopLDeviceErr";
    case error_lcard::RequestBufferStreamErr:
        return "RequestBufferStreamErr";
    case error_lcard::FillDAQparametersErr:
        return "FillDAQparametersErr";
    case error_lcard::SetParametersStreamErr:
        return "SetParametersStreamErr";
    case error_lcard::GetParameterErr:
        return "GetParameterErr";
    case error_lcard::SetRegFreqErr:
        return "SetRegFreqErr";
    case error_lcard::NotFullCadrInHalfBuffer:
        return "NotFullCadrInHalfBuffer";
    default:
        return "Invalid error code";
    }
}

}
