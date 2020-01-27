#pragma once

#include <errors.hpp>

namespace ros {

enum class error_devices_depthometer
{
    generic = 1,
    write_err,
    read_err,
    invalid_bcc,
    invalid_error,
    invalid_ack
};

enum class error_devices_mover
{
    generic = 1,
    invalid_direction,
};

enum class error_devices_rotator
{
    generic = 1,
    invalid_direction,
    invalid_speed,
};

}

REGISTER_ERROR_CODES(ros, error_devices_depthometer);
REGISTER_ERROR_CODES(ros, error_devices_mover);
REGISTER_ERROR_CODES(ros, error_devices_rotator);
