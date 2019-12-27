#include <SlicerImpl.h>

#include <cassert>

namespace ros { namespace pike { namespace logic {

SliceMsr SlicerImpl::Read(const std::atomic_bool& cancel_token, SlicerReadOutput* output)
{
    assert(output != nullptr);

    const size_t MaxStepInEnder{1};

    SliceMsr res;

    // поворот в крайнее левое положение
    pike_->rotator()->SetDirection(ros::devices::RotatorDirection::CCW);
    pike_->rotator()->SetSpeed(ros::devices::RotatorSpeed::High);

    // TODO: не более числа шагов, нужного на полный оборот (вдруг ender не работает)
    // TODO: "дожимать" несколько шагов после срабатывания ender
    while (true) {
        // читаем сразу оба ender (за одно чтение ttl_in)
        pike_->ReadAndUpdateTtlIn();
        const bool ender1 = pike_->ender1()->Get();
        const bool ender2 = pike_->ender2()->Get();

        output->TtlInTick(ender1, ender2);

        // TODO: что возвращает датчик в "активном" состоянии? - предполагаем, что 1
        if (ender1) {
            break;
        }

        // TODO: use Step
        pike_->rotator()->Rotate();

        if (cancel_token) {
            return res;
        }
    }

    // получение положения в пространстве
    res.inclio_angle = pike_->inclinometer()->Get();

    // измерение, поворачивая в крайнее правое положение
    pike_->rotator()->SetDirection(ros::devices::RotatorDirection::CW);
    pike_->rotator()->SetSpeed(ros::devices::RotatorSpeed::Low);

    assert(pike_->rotator()->StepsIn360() > 0);
    const double_t angle_per_step = 360.0 / pike_->rotator()->StepsIn360();

    // TODO: не более числа шагов, нужного на полный оборот (вдруг ender не работает)
    double_t angle{0};  // TODO: начинаем с нулевого угла?
    size_t steps_in_ender2{0};
    while (true) {
        // читаем сразу оба ender (за одно чтение ttl_in)
        pike_->ReadAndUpdateTtlIn();
        const bool ender1 = pike_->ender1()->Get();
        const bool ender2 = pike_->ender2()->Get();

        output->TtlInTick(ender1, ender2);

        // TODO: что возвращает датчик в "активном" состоянии? - предполагаем, что 1
        if (ender2) {
            // "дожимаем" несколько шагов после срабатывания ender
            if (steps_in_ender2++ >= MaxStepInEnder) {
                break;
            }
        } else {
            assert(steps_in_ender2 == 0);  // ender выставился, а потом сбросился
        }

        const int16_t depth = pike_->depthometer()->Read();

        res.angles.emplace_back(angle);
        res.depths.emplace_back(depth);

        output->SliceTick(angle, depth);

        // TODO: use Step
        pike_->rotator()->Rotate();

        angle += angle_per_step;

        if (cancel_token) {
            return res;
        }
    }

    //pike_->rotator()->Disable();

    res.ok = true;

    return res;
}

}}}
