#include <Inclinometer.h>

#define _USE_MATH_DEFINES
#include <cmath>

namespace ros { namespace devices {

void Inclinometer::Update(const std::vector<int16_t>& values)
{
    std::vector<int16_t> x_values;
    std::vector<int16_t> y_values;

    double_t sin_fi_x;
    double_t sin_fi_y;
    double_t res;

    if (sin_fi_x < sqrt(2) / 2) {
        if (sin_fi_y > 0) {
            res = asin(sin_fi_x) * 180 / M_PI;
        } else {
            if (sin_fi_x > 0) {
                res = 180 - asin(sin_fi_x) * 180 / M_PI;
            } else {
                // TODO:
            }
        }
    } else {
        if (sin_fi_x > 0) {
            res = 90 - asin(sin_fi_y) * 180 / M_PI;
        } else {
            res = -90 + asin(sin_fi_y) * 180 / M_PI;
        }
    }
}

int32_t Inclinometer::Get()
{
    return 0;
}

}}
