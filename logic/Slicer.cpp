#include <Slicer.h>

namespace ros { namespace pike { namespace logic {

void Slicer::Slice()
{
    // ������� � ������� ����� ���������
    pike_->rotator()->SetDirection(ros::devices::RotatorDirection::CCW);
    pike_->rotator()->SetSpeed(ros::devices::RotatorSpeed::High);

    // TODO: �� ����� ����� �����, ������� �� ������ ������ (����� ender �� ��������)
    // TODO: "��������" ��������� ����� ����� ������������ ender
    while (!pike_->ender1()->Read()) {
        pike_->rotator()->Rotate();
    }

    // ��������� ��������� � ������������
    //pike_->

    // ���������, ����������� � ������� ������ ���������
    pike_->rotator()->SetDirection(ros::devices::RotatorDirection::CW);
    pike_->rotator()->SetSpeed(ros::devices::RotatorSpeed::Low);

    // TODO: �� ����� ����� �����, ������� �� ������ ������ (����� ender �� ��������)
    // TODO: "��������" ��������� ����� ����� ������������ ender
    while (!pike_->ender2()->Read()) {
        const auto depth = pike_->depthometer()->Read();

        pike_->rotator()->Rotate();
    }

    pike_->rotator()->Disable();
}

}}}
