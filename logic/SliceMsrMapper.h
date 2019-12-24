#pragma once

#include <Slicer.h>

namespace ros { namespace pike { namespace logic {

class SliceMsrMapper
{
public:
    virtual void Save(const SliceMsr& slice_msr) = 0;
};

}}}
