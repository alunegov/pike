#include <error_logic.hpp>

namespace ros {

std::string error_logic_messages::message(int ev) const
{
    return "Invalid error code";
    /*switch (static_cast<error_logic>(ev)) {
    default:
        return "Invalid error code";
    }*/
}

}
