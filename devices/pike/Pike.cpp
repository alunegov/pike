#include <Pike.h>

namespace ros { namespace devices {

Pike::~Pike()
{
    delete ender1_;
    delete ender2_;
    delete rotator_;
    delete mover_;
    delete odometer_;
    delete inclinometer_;
    delete depthometer_;

    // удаляемые выше зависят от daq_
    delete daq_;
}

}}
