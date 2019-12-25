#pragma once

#include <Remote.h>

namespace ros { namespace pike { namespace logic {

class RemoteImpl : public Remote
{
public:
    ~RemoteImpl() override;

    // Remote

    void SetOutput(RemoteOutput* output) override;

    void Start() override;

    void Stop() override;

private:
    RemoteOutput* _output{nullptr};
};

}}}
