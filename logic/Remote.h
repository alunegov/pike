#pragma once

namespace ros { namespace pike { namespace logic {

class RemoteOutput
{
public:
    virtual ~RemoteOutput() = default;
};

class Remote
{
public:
    virtual ~Remote() = default;

    virtual void SetOutput(RemoteOutput* output) = 0;

    virtual void Start() = 0;

    virtual void Stop() = 0;
};

}}}
