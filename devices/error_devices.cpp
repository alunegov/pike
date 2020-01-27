#include <error_devices.hpp>

namespace ros {

std::string error_devices_depthometer_messages::message(int ev) const
{
    switch (static_cast<error_devices_depthometer>(ev)) {
    case error_devices_depthometer::write_err:
        return "Write error";
    case error_devices_depthometer::read_err:
        return "Read error";
    case error_devices_depthometer::invalid_bcc:
        return "Invalid BCC";
    case error_devices_depthometer::invalid_error:
        return "Invalid error";
    case error_devices_depthometer::invalid_ack:
        return "Invalid ACK";
    default:
        return "Invalid error code";
    }
}

std::string error_devices_mover_messages::message(int ev) const
{
    switch (static_cast<error_devices_mover>(ev)) {
    case error_devices_mover::invalid_direction:
        return "Invalid direction";
    default:
        return "Invalid error code";
    }
}

std::string error_devices_rotator_messages::message(int ev) const
{
    switch (static_cast<error_devices_rotator>(ev)) {
    case error_devices_rotator::invalid_direction:
        return "Invalid direction";
    case error_devices_rotator::invalid_speed:
        return "Invalid speed";
    default:
        return "Invalid error code";
    }
}

}
