#pragma once

#include <errors.hpp>

namespace ros {

enum class error_logic
{
    generic = 1,
};

}

REGISTER_ERROR_CODES(ros, error_logic);
