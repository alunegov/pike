#include <SlicerImpl.h>

#include <cassert>

namespace ros { namespace pike { namespace logic {

void SlicerImpl::Read(const std::atomic_bool& cancel_token, SlicerReadOutput* output)
{
    assert(output != nullptr);

    const size_t MaxStepInEnder{1};

    // поворот в крайнее левое положение
    pike_->rotator()->SetDirection(ros::devices::RotatorDirection::CCW);
    pike_->rotator()->SetSpeed(ros::devices::RotatorSpeed::High);

    // TODO: не более числа шагов, нужного на полный оборот (вдруг ender не работает)
    // TODO: "дожимать" несколько шагов после срабатывания ender
    while (true) {
        const bool ender1 = pike_->ender1()->Read();
        const bool ender2 = pike_->ender2()->Read();

        output->TtlInTick(ender1, ender2);

        // TODO: что возвращает датчик в "активном" состоянии? - предполагаем, что 1
        if (ender1) {
            break;
        }

        pike_->rotator()->Rotate();

        if (cancel_token) {
            return;
        }
    }

    // получение положения в пространстве
    const double_t angle = pike_->inclinometer()->Get();

    // измерение, поворачивая в крайнее правое положение
    pike_->rotator()->SetDirection(ros::devices::RotatorDirection::CW);
    pike_->rotator()->SetSpeed(ros::devices::RotatorSpeed::Low);

    // TODO: не более числа шагов, нужного на полный оборот (вдруг ender не работает)
    size_t i{0};
    size_t steps_in_ender2{0};
    while (true) {
        const bool ender1 = pike_->ender1()->Read();
        const bool ender2 = pike_->ender2()->Read();

        output->TtlInTick(ender1, ender2);

        // TODO: что возвращает датчик в "активном" состоянии? - предполагаем, что 1
        if (ender2) {
            // "дожимаем" несколько шагов после срабатывания ender
            if (steps_in_ender2++ >= MaxStepInEnder) {
                break;
            }
        } else {
            assert(steps_in_ender2 == 0);
        }

        const int16_t depth = pike_->depthometer()->Read();

        output->SliceTick(i++, depth);

        pike_->rotator()->Rotate();

        if (cancel_token) {
            return;
        }
    }

    //pike_->rotator()->Disable();
}

}}}
