#pragma once

#include <cstdint>

namespace ros { namespace pike { namespace modules {

class MainView {
public:
    virtual void SetDepth(int16_t value) = 0;
};

}}}
