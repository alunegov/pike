#include <Inclinometer.h>

#define _USE_MATH_DEFINES
#include <math.h>

#include <RosMath.h>

namespace ros { namespace devices {

void Inclinometer::Update(const std::vector<int16_t>& values)
{
    //double_t x_rms = Rms_AdcRaw(values.data(), 1024, 4, 0, 0);
    //double_t y_rms = Rms_AdcRaw(values.data(), 1024, 4, 1, 0);

    double_t sin_fi_x{0};
    double_t sin_fi_y{0};

    double_t new_angle;

    if (sin_fi_x < sqrt(2) / 2) {
        if (sin_fi_y > 0) {
            new_angle = asin(sin_fi_x) * 180 / M_PI;
        } else {
            if (sin_fi_x > 0) {
                new_angle = 180 - asin(sin_fi_x) * 180 / M_PI;
            } else {
                // TODO: на блок-схеме этот кусок не влез
                new_angle = 0;
            }
        }
    } else {
        if (sin_fi_x > 0) {
            new_angle = 90 - asin(sin_fi_y) * 180 / M_PI;
        } else {
            new_angle = -90 + asin(sin_fi_y) * 180 / M_PI;
        }
    }

    angle_ = new_angle;
}

double_t Inclinometer::Get()
{
    return angle_;
}

}}
