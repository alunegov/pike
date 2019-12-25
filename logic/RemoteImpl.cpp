#include <RemoteImpl.h>

#include <cassert>

namespace ros { namespace pike { namespace logic {

RemoteImpl::~RemoteImpl()
{}

void RemoteImpl::SetOutput(RemoteOutput* output)
{
    assert(output != nullptr);
    _output = output;
}

void RemoteImpl::Start()
{}

void RemoteImpl::Stop()
{}

}}}
