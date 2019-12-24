#pragma once

#include <SliceMsrMapper.h>

namespace ros { namespace pike { namespace logic {

class SliceMsrMapperImpl : public SliceMsrMapper
{
public:
    void Save(const SliceMsr& slice_msr) override;
};

}}}
