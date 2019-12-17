#include <SlicerImpl.h>

namespace ros { namespace pike { namespace logic {

void SlicerImpl::Read(const std::atomic_bool& cancel_token, const std::function<CallbackFunc>& callback)
{
    // поворот в крайнее левое положение
    pike_->rotator()->SetDirection(ros::devices::RotatorDirection::CCW);
    pike_->rotator()->SetSpeed(ros::devices::RotatorSpeed::High);

    // TODO: что возвращает датчик в "активном" состоянии - предполагаем, что 1
    // TODO: не более числа шагов, нужного на полный оборот (вдруг ender не работает)
    // TODO: "дожимать" несколько шагов после срабатывания ender
    while (!pike_->ender1()->Read()) {
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

    // TODO: что возвращает датчик в "активном" состоянии - предполагаем, что 1
    // TODO: не более числа шагов, нужного на полный оборот (вдруг ender не работает)
    // TODO: "дожимать" несколько шагов после срабатывания ender
    while (!pike_->ender2()->Read()) {
        const int16_t depth = pike_->depthometer()->Read();

        callback(0, depth);

        pike_->rotator()->Rotate();

        if (cancel_token) {
            return;
        }
    }

    //pike_->rotator()->Disable();
}

}}}
