#include <MainViewImpl.h>

#include <QtWidgets/QVBoxLayout>

#include <Mover.h>
#include <Rotator.h>

namespace ros { namespace pike { namespace ui {

MainViewImpl::MainViewImpl(ros::pike::modules::MainPresenter* presenter) :
    presenter_{presenter}
{
    distance_label_ = new QLabel;
    distance_label_->setText("distance");

    angle_label_ = new QLabel;
    angle_label_->setText("angle");

    depth_label_ = new QLabel;
    depth_label_->setText("depth");

    ender1_label_ = new QLabel;
    ender1_label_->setText("ender1");

    ender2_label_ = new QLabel;
    ender2_label_->setText("ender2");

    move_forward_button_ = new QPushButton;
    move_forward_button_->setText("Fwd");
    QObject::connect(move_forward_button_, &QPushButton::pressed, this, [=]() {
        presenter_->StartMoving(ros::devices::MoverDirection::Forward);
    });
    QObject::connect(move_forward_button_, &QPushButton::released, this, [=]() {
        presenter_->StopMoving();
    });

    move_backward_button_ = new QPushButton;
    move_backward_button_->setText("Back");
    QObject::connect(move_backward_button_, &QPushButton::pressed, this, [=]() {
        presenter_->StartMoving(ros::devices::MoverDirection::Backward);
    });
    QObject::connect(move_backward_button_, &QPushButton::released, this, [=]() {
        presenter_->StopMoving();
    });

    rotate_ccw_button_ = new QPushButton;
    rotate_ccw_button_->setText("CCW");
    QObject::connect(rotate_ccw_button_, &QPushButton::pressed, this, [=]() {
        presenter_->StartRotation(ros::devices::RotatorDirection::CCW);
    });
    QObject::connect(rotate_ccw_button_, &QPushButton::released, this, [=]() {
        presenter_->StopRotation();
    });

    rotate_cw_button_ = new QPushButton;
    rotate_cw_button_->setText("CW");
    QObject::connect(rotate_cw_button_, &QPushButton::pressed, this, [=]() {
        presenter_->StartRotation(ros::devices::RotatorDirection::CW);
    });
    QObject::connect(rotate_cw_button_, &QPushButton::released, this, [=]() {
        presenter_->StopRotation();
    });

    slice_button_ = new QPushButton;
    slice_button_->setText("Slice");
    QObject::connect(slice_button_, &QPushButton::clicked, this, [=]() {
        presenter_->SliceClicked();
    });

    //
    auto verticalLayout = new QVBoxLayout;
    verticalLayout->addWidget(distance_label_);
    verticalLayout->addWidget(angle_label_);
    verticalLayout->addWidget(depth_label_);
    verticalLayout->addWidget(ender1_label_);
    verticalLayout->addWidget(ender2_label_);
    verticalLayout->addWidget(move_forward_button_);
    verticalLayout->addWidget(move_backward_button_);
    verticalLayout->addWidget(rotate_ccw_button_);
    verticalLayout->addWidget(rotate_cw_button_);
    verticalLayout->addWidget(slice_button_);

    setLayout(verticalLayout);

    //
    presenter_->SetView(this);

    presenter_->OnShow();
}

MainViewImpl::~MainViewImpl()
{
    if (presenter_ != nullptr) {
        presenter_->SetView(nullptr);
        delete presenter_;
    }
}

void MainViewImpl::SetDistance(double_t value)
{
    distance_label_->setText(QString::number(value));
}

void MainViewImpl::SetAngle(double_t value)
{
    angle_label_->setText(QString::number(value));
}

void MainViewImpl::SetDepth(int16_t value)
{
    depth_label_->setText(QString::number(value));
}

void MainViewImpl::UpdateSliceDepth(double_t angle, int16_t depth)
{
    depth_label_->setText(QString{"%1 at %2"}.arg(depth).arg(angle));
}

void MainViewImpl::SetEnders(bool ender1, bool ender2)
{
    ender1_label_->setText(QString::number(ender1));
    ender2_label_->setText(QString::number(ender2));
}

void MainViewImpl::SetAdcChannels(const std::vector<uint16_t>& channels, const int16_t* values, size_t values_count,
        double_t adc_to_volt)
{

}

}}}
