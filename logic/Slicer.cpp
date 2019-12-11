#include <Slicer.h>

namespace ros { namespace pike { namespace logic {

void Slicer::Slice()
{
    // поворот в крайнее левое положение
    pike_->rotator()->SetDirection(ros::devices::RotatorDirection::CCW);
    pike_->rotator()->SetSpeed(ros::devices::RotatorSpeed::High);

    // TODO: не более числа шагов, нужного на полный оборот (вдруг ender не работает)
    // TODO: "дожимать" несколько шагов после срабатывания ender
    while (!pike_->ender1()->Read()) {
        pike_->rotator()->Rotate();
    }

    // измерение положения в пространстве
    //pike_->

    // измерение, поворачивая в крайнее правое положение
    pike_->rotator()->SetDirection(ros::devices::RotatorDirection::CW);
    pike_->rotator()->SetSpeed(ros::devices::RotatorSpeed::Low);

    // TODO: не более числа шагов, нужного на полный оборот (вдруг ender не работает)
    // TODO: "дожимать" несколько шагов после срабатывания ender
    while (!pike_->ender2()->Read()) {
        const auto depth = pike_->depthometer()->Read();

        pike_->rotator()->Rotate();
    }

    pike_->rotator()->Disable();
}

}}}
