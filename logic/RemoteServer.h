#pragma once

namespace ros { namespace pike { namespace logic {

enum class MotionDirection
{
    No,
    Inc,
    Dec,
};

class RemoteServerOutput
{
public:
    virtual ~RemoteServerOutput() = default;

    virtual void RemoteStartMovement(MotionDirection dir) = 0;

    virtual void RemoteStopMovement() = 0;

    virtual void RemoteStartRotation(MotionDirection dir) = 0;

    virtual void RemoteStopRotation() = 0;
};

class RemoteServer
{
public:
    virtual ~RemoteServer() = default;

    virtual void SetOutput(RemoteServerOutput* output) = 0;

    virtual void Start() = 0;

    virtual void Stop() = 0;
};

}}}
