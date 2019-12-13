#include <Slicer.h>

namespace ros { namespace pike { namespace logic {

void Slicer::Read(const std::atomic_bool& cancel_token, std::function<CallbackFunc> callback)
{
    // ������� � ������� ����� ���������
    pike_->rotator()->SetDirection(ros::devices::RotatorDirection::CCW);
    pike_->rotator()->SetSpeed(ros::devices::RotatorSpeed::High);

    // TODO: �� ����� ����� �����, ������� �� ������ ������ (����� ender �� ��������)
    // TODO: "��������" ��������� ����� ����� ������������ ender
    while (!pike_->ender1()->Read()) {
        pike_->rotator()->Rotate();

        if (cancel_token) {
            return;
        }
    }

    // ��������� ��������� � ������������
    const auto angle = pike_->inclinometer()->Get();

    // ���������, ����������� � ������� ������ ���������
    pike_->rotator()->SetDirection(ros::devices::RotatorDirection::CW);
    pike_->rotator()->SetSpeed(ros::devices::RotatorSpeed::Low);

    // TODO: �� ����� ����� �����, ������� �� ������ ������ (����� ender �� ��������)
    // TODO: "��������" ��������� ����� ����� ������������ ender
    while (!pike_->ender2()->Read()) {
        const auto depth = pike_->depthometer()->Read();

        callback(0, depth);

        pike_->rotator()->Rotate();

        if (cancel_token) {
            return;
        }
    }

    //pike_->rotator()->Disable();
}

}}}
