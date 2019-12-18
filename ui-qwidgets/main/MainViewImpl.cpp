#include <MainViewImpl.h>

#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>

#include <Mover.h>
#include <Rotator.h>

namespace ros { namespace pike { namespace ui {

MainViewImpl::MainViewImpl(ros::pike::modules::MainPresenter* presenter) :
    presenter_{presenter}
{
    camera_viewport_label_ = new QLabel;
    camera_viewport_label_->setText("camera_viewport");

    distance_label_ = new QLabel;
    distance_label_->setText("distance");

    angle_label_ = new QLabel;
    angle_label_->setText("angle");
    //angle_label_->setMinimumWidth(100);
    angle_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    slice_viewport_label_ = new QLabel;
    slice_viewport_label_->setText("slice_viewport");
    slice_viewport_label_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);

    depth_label_ = new QLabel;
    depth_label_->setText("depth");

    ender1_label_ = new QLabel;
    ender1_label_->setText("ender1");

    ender2_label_ = new QLabel;
    ender2_label_->setText("ender2");

    move_forward_button_ = new QPushButton;
    move_forward_button_->setText("fwd");
    QObject::connect(move_forward_button_, &QPushButton::pressed, this, [=]() {
        presenter_->StartMoving(ros::devices::MoverDirection::Forward);
    });
    QObject::connect(move_forward_button_, &QPushButton::released, this, [=]() {
        presenter_->StopMoving();
    });

    move_backward_button_ = new QPushButton;
    move_backward_button_->setText("back");
    QObject::connect(move_backward_button_, &QPushButton::pressed, this, [=]() {
        presenter_->StartMoving(ros::devices::MoverDirection::Backward);
    });
    QObject::connect(move_backward_button_, &QPushButton::released, this, [=]() {
        presenter_->StopMoving();
    });

    rotate_ccw_button_ = new QPushButton;
    rotate_ccw_button_->setText("ccw");
    QObject::connect(rotate_ccw_button_, &QPushButton::pressed, this, [=]() {
        presenter_->StartRotation(ros::devices::RotatorDirection::CCW);
    });
    QObject::connect(rotate_ccw_button_, &QPushButton::released, this, [=]() {
        presenter_->StopRotation();
    });

    rotate_cw_button_ = new QPushButton;
    rotate_cw_button_->setText("cw");
    QObject::connect(rotate_cw_button_, &QPushButton::pressed, this, [=]() {
        presenter_->StartRotation(ros::devices::RotatorDirection::CW);
    });
    QObject::connect(rotate_cw_button_, &QPushButton::released, this, [=]() {
        presenter_->StopRotation();
    });

    slice_button_ = new QPushButton;
    slice_button_->setText("slice");
    QObject::connect(slice_button_, &QPushButton::clicked, this, [=]() {
        presenter_->SliceClicked();
    });

    camera1_button_ = new QPushButton;
    camera1_button_->setText("camera1");
    QObject::connect(camera1_button_, &QPushButton::clicked, this, [=]() {
        
    });

    camera2_button_ = new QPushButton;
    camera2_button_->setText("camera2");
    QObject::connect(camera2_button_, &QPushButton::clicked, this, [=]() {
        
    });

    rec_button_ = new QPushButton;
    rec_button_->setText("rec");
    QObject::connect(rec_button_, &QPushButton::clicked, this, [=]() {
        
    });

    photo_button_ = new QPushButton;
    photo_button_->setText("photo");
    QObject::connect(photo_button_, &QPushButton::clicked, this, [=]() {
        
    });

    dest_path_edit_ = new QLineEdit;
    dest_path_edit_->setText("dest_path");

    // layout
    auto slice_layout = new QVBoxLayout;
    slice_layout->addWidget(slice_viewport_label_);
    auto slice_bottom_layout = new QHBoxLayout;
    slice_bottom_layout->addWidget(ender1_label_);
    slice_bottom_layout->addWidget(depth_label_);
    slice_bottom_layout->addWidget(ender2_label_);
    slice_layout->addLayout(slice_bottom_layout);

    auto camera_and_slice_layout = new QHBoxLayout;
    camera_and_slice_layout->addWidget(camera_viewport_label_);
    camera_and_slice_layout->addLayout(slice_layout);

    auto move_buttons_layout = new QGridLayout;
    move_buttons_layout->addWidget(move_forward_button_, 0, 1);
    move_buttons_layout->addWidget(move_backward_button_, 2, 1);
    move_buttons_layout->addWidget(rotate_ccw_button_, 1, 0);
    move_buttons_layout->addWidget(rotate_cw_button_, 1, 3);

    auto inclio_and_move_buttons_layout = new QHBoxLayout;
    inclio_and_move_buttons_layout->addWidget(angle_label_);
    inclio_and_move_buttons_layout->addLayout(move_buttons_layout);

    auto distance_and_inclio_and_move_buttons_layout = new QVBoxLayout;
    distance_and_inclio_and_move_buttons_layout->addWidget(distance_label_);
    distance_and_inclio_and_move_buttons_layout->addLayout(inclio_and_move_buttons_layout);

    auto buttons_layout = new QVBoxLayout;
    buttons_layout->addWidget(slice_button_);
    auto l21  = new QHBoxLayout;
    l21->addWidget(camera1_button_);
    l21->addWidget(camera2_button_);
    buttons_layout->addLayout(l21);
    auto l22  = new QHBoxLayout;
    l22->addWidget(rec_button_);
    l22->addWidget(photo_button_);
    buttons_layout->addLayout(l22);
    buttons_layout->addWidget(dest_path_edit_);

    auto bottom_layout = new QHBoxLayout;
    bottom_layout->addLayout(distance_and_inclio_and_move_buttons_layout);
    //bottom_layout->addStretch();
    bottom_layout->addLayout(buttons_layout);

    auto rootLayout = new QVBoxLayout;
    rootLayout->addLayout(camera_and_slice_layout);
    rootLayout->addLayout(bottom_layout);

    setLayout(rootLayout);

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
