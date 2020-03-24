#include <PikeImpl.h>

namespace ros { namespace devices {

PikeImpl::~PikeImpl()
{
    delete ender1_;
    delete ender2_;
    delete rotator_;
    delete mover_;
    delete odometer_;
    delete inclinometer_;
    delete depthometer_;

    // ��������� ���� ������� �� daq_
    delete daq_;
}

}}
