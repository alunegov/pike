#include <error_logic.hpp>

namespace ros {

std::string error_logic_messages::message(int ev) const
{
    (void)ev;
    return "Invalid error code";
    /*switch (static_cast<error_logic>(ev)) {
    default:
        return "Invalid error code";
    }*/
}

}
