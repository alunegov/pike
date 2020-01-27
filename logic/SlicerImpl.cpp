#include <SlicerImpl.h>

#include <cassert>

namespace ros { namespace pike { namespace logic {

SliceMsr SlicerImpl::Read(const std::atomic_bool& cancel_token, SlicerReadOutput* output)
{
    assert(output != nullptr);

    const size_t MaxStepInEnder{1};

    SliceMsr res;

    const auto rotator_enable_opt = pike_->rotator()->Enable();
    if (!rotator_enable_opt) {
        // TODO: log and return?
        res.ec = rotator_enable_opt.error();
        return res;
    }

    // поворот в крайнее левое положение
    pike_->rotator()->SetDirection(ros::devices::RotatorDirection::CCW);
    pike_->rotator()->SetSpeed(ros::devices::RotatorSpeed::High);
    auto rotator_prestep_opt = pike_->rotator()->PreStep();
    if (!rotator_prestep_opt) {
        // TODO: log and return?
        res.ec = rotator_prestep_opt.error();
        return res;
    }

    // TODO: не более числа шагов, нужного на полный оборот (вдруг ender не работает)
    // TODO: "дожимать" несколько шагов после срабатывания ender
    while (true) {
        // читаем сразу оба ender (за одно чтение ttl_in)
        const auto ttlin_opt = pike_->ReadAndUpdateTtlIn();
        if (!ttlin_opt) {
            // TODO: log and return?
            res.ec = ttlin_opt.error();
            return res;
        }
        const bool ender1 = pike_->ender1()->Get();
        const bool ender2 = pike_->ender2()->Get();

        output->TtlInTick(ender1, ender2);

        // TODO: что возвращает датчик в "активном" состоянии? - предполагаем, что 1
        if (ender1) {
            break;
        }

        const auto rotate_opt = pike_->rotator()->Step();
        if (!rotate_opt) {
            // TODO: log and return?
            res.ec = rotate_opt.error();
            return res;
        }

        if (cancel_token) {
            return res;
        }
    }

    // получение положения в пространстве
    res.inclio_angle = pike_->inclinometer()->Get();

    // измерение, поворачивая в крайнее правое положение
    pike_->rotator()->SetDirection(ros::devices::RotatorDirection::CW);
    pike_->rotator()->SetSpeed(ros::devices::RotatorSpeed::Low);
    rotator_prestep_opt = pike_->rotator()->PreStep();
    if (!rotator_prestep_opt) {
        // TODO: log and return?
        res.ec = rotator_prestep_opt.error();
        return res;
    }

    assert(pike_->rotator()->StepsIn360() > 0);
    const double_t angle_per_step = 360.0 / pike_->rotator()->StepsIn360();

    // TODO: не более числа шагов, нужного на полный оборот (вдруг ender не работает)
    double_t angle{0};  // TODO: начинаем с нулевого угла?
    size_t steps_in_ender2{0};
    while (true) {
        // читаем сразу оба ender (за одно чтение ttl_in)
        const auto ttlin_opt = pike_->ReadAndUpdateTtlIn();
        if (!ttlin_opt) {
            // TODO: log and return?
            res.ec = ttlin_opt.error();
            return res;
        }
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

        const auto depth = pike_->depthometer()->Read();
        if (!depth) {
            // TODO: log and return?
            res.ec = depth.error();
            return res;
        }

        res.angles.emplace_back(angle);
        res.depths.emplace_back(depth.value());

        output->SliceTick(angle, depth.value());

        const auto rotate_opt = pike_->rotator()->Step();
        if (!rotate_opt) {
            // TODO: log and return?
            res.ec = rotate_opt.error();
            return res;
        }

        angle += angle_per_step;

        if (cancel_token) {
            return res;
        }
    }

    /*const auto rotator_disable_opt = pike_->rotator()->Disable();
    if (!rotator_disable_opt) {
        // TODO: log
        res.ec = rotator_disable_opt.error();
        return res;
    }*/

    res.ok = true;

    return res;
}

}}}
