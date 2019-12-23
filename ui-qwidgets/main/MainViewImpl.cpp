#include <MainViewImpl.h>

#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>

#include <Mover.h>
#include <Rotator.h>

namespace ros { namespace pike { namespace ui {

const size_t distance_viewport_height{40};
const size_t min_viewport_size{100};

MainViewImpl::MainViewImpl(ros::pike::modules::MainPresenter* presenter) :
    presenter_{presenter}
{
    camera_viewport_ = new CameraWidget;
    camera_viewport_->setMinimumSize(min_viewport_size, min_viewport_size);
    camera_viewport_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);

    distance_viewport_ = new DistanceWidget;
    distance_viewport_->setMinimumSize(min_viewport_size, distance_viewport_height);
    distance_viewport_->setMaximumSize(width() * 6 / 10, distance_viewport_height);
    distance_viewport_->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Preferred);

    inclio_viewport_ = new InclioWidget;
    inclio_viewport_->setMinimumSize(min_viewport_size, min_viewport_size);
    //inclio_viewport_->setMaximumHeight(200);
    //inclio_viewport_->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Preferred);

    slice_viewport_ = new SliceWidget;
    slice_viewport_->setMinimumSize(min_viewport_size, min_viewport_size);
    //slice_viewport_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    slice_viewport_->SetDummySlice();

    depth_label_ = new QLabel;
    depth_label_->setText("depth");
    depth_label_->setAlignment(Qt::AlignCenter);

    ender1_label_ = new QLabel;
    ender1_label_->setText("ender1");
    ender1_label_->setAlignment(Qt::AlignCenter);

    ender2_label_ = new QLabel;
    ender2_label_->setText("ender2");
    ender2_label_->setAlignment(Qt::AlignCenter);

    move_forward_button_ = new QPushButton;
    move_forward_button_->setText("fwd");
    move_forward_button_->setObjectName("motion");
    QObject::connect(move_forward_button_, &QPushButton::pressed, this, [=]() {
        presenter_->StartMoving(ros::devices::MoverDirection::Forward);
    });
    QObject::connect(move_forward_button_, &QPushButton::released, this, [=]() {
        presenter_->StopMoving();
    });

    move_backward_button_ = new QPushButton;
    move_backward_button_->setText("back");
    move_backward_button_->setObjectName("motion");
    QObject::connect(move_backward_button_, &QPushButton::pressed, this, [=]() {
        presenter_->StartMoving(ros::devices::MoverDirection::Backward);
    });
    QObject::connect(move_backward_button_, &QPushButton::released, this, [=]() {
        presenter_->StopMoving();
    });

    rotate_ccw_button_ = new QPushButton;
    rotate_ccw_button_->setText("ccw");
    rotate_ccw_button_->setObjectName("motion");
    QObject::connect(rotate_ccw_button_, &QPushButton::pressed, this, [=]() {
        presenter_->StartRotation(ros::devices::RotatorDirection::CCW);
    });
    QObject::connect(rotate_ccw_button_, &QPushButton::released, this, [=]() {
        presenter_->StopRotation();
    });

    rotate_cw_button_ = new QPushButton;
    rotate_cw_button_->setText("cw");
    rotate_cw_button_->setObjectName("motion");
    QObject::connect(rotate_cw_button_, &QPushButton::pressed, this, [=]() {
        presenter_->StartRotation(ros::devices::RotatorDirection::CW);
    });
    QObject::connect(rotate_cw_button_, &QPushButton::released, this, [=]() {
        presenter_->StopRotation();
    });

    slice_button_ = new QPushButton;
    slice_button_->setText("slice");
    slice_button_->setCheckable(true);
    slice_button_->setObjectName("record");
    QObject::connect(slice_button_, &QPushButton::toggled, this, [=](bool checked) {
        if (checked) {
            presenter_->StartSlice();
        } else {
            presenter_->StopSlice();
        }
    });

    camera1_button_ = new QPushButton;
    camera1_button_->setText("camera1");
    QObject::connect(camera1_button_, &QPushButton::clicked, this, [=]() {
        presenter_->Camera1Clicked();
    });

    camera2_button_ = new QPushButton;
    camera2_button_->setText("camera2");
    QObject::connect(camera2_button_, &QPushButton::clicked, this, [=]() {
        presenter_->Camera2Clicked();
    });

    rec_button_ = new QPushButton;
    rec_button_->setText("rec");
    rec_button_->setCheckable(true);
    rec_button_->setObjectName("record");
    QObject::connect(rec_button_, &QPushButton::toggled, this, [=](bool checked) {
        if (checked) {
            presenter_->StartRec();
        } else {
            presenter_->StopRec();
        }
    });

    photo_button_ = new QPushButton;
    photo_button_->setText("photo");
    QObject::connect(photo_button_, &QPushButton::clicked, this, [=]() {
        presenter_->PhotoClicked();
    });

    dest_path_edit_ = new QLineEdit;
    dest_path_edit_->setText("dest_path");

    // layout
    auto camera_and_slice_layout = new QHBoxLayout;
    camera_and_slice_layout->addWidget(camera_viewport_);
    camera_and_slice_layout->addWidget(slice_viewport_);

    auto distance_and_depth_layout = new QHBoxLayout;
    distance_and_depth_layout->addWidget(distance_viewport_);
    distance_and_depth_layout->addWidget(ender1_label_);
    distance_and_depth_layout->addWidget(depth_label_);
    distance_and_depth_layout->addWidget(ender2_label_);

    auto move_buttons_layout = new QGridLayout;
    move_buttons_layout->addWidget(move_forward_button_, 0, 1);
    move_buttons_layout->addWidget(move_backward_button_, 2, 1);
    move_buttons_layout->addWidget(rotate_ccw_button_, 1, 0);
    move_buttons_layout->addWidget(rotate_cw_button_, 1, 3);

    auto buttons_layout = new QVBoxLayout;
    //buttons_layout->addStretch(2);
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
    //buttons_layout->addStretch(1);

    auto bottom_layout = new QHBoxLayout;
    bottom_layout->addWidget(inclio_viewport_);
    bottom_layout->addStretch();
    bottom_layout->addLayout(move_buttons_layout);
    bottom_layout->addStretch();
    bottom_layout->addLayout(buttons_layout);

    /*auto slice_layout = new QVBoxLayout;
    slice_layout->addWidget(slice_viewport_);
    auto slice_bottom_layout = new QHBoxLayout;
    slice_bottom_layout->addWidget(ender1_label_);
    slice_bottom_layout->addWidget(depth_label_);
    slice_bottom_layout->addWidget(ender2_label_);
    slice_layout->addLayout(slice_bottom_layout);

    auto camera_and_slice_layout = new QHBoxLayout;
    camera_and_slice_layout->addWidget(camera_viewport_);
    camera_and_slice_layout->addLayout(slice_layout);

    auto move_buttons_layout = new QGridLayout;
    move_buttons_layout->addWidget(move_forward_button_, 0, 1);
    move_buttons_layout->addWidget(move_backward_button_, 2, 1);
    move_buttons_layout->addWidget(rotate_ccw_button_, 1, 0);
    move_buttons_layout->addWidget(rotate_cw_button_, 1, 3);

    auto inclio_and_move_buttons_layout = new QHBoxLayout;
    inclio_and_move_buttons_layout->addWidget(inclio_viewport_);
    inclio_and_move_buttons_layout->addLayout(move_buttons_layout);

    auto distance_and_inclio_and_move_buttons_layout = new QVBoxLayout;
    distance_and_inclio_and_move_buttons_layout->addWidget(distance_viewport_);
    distance_and_inclio_and_move_buttons_layout->addLayout(inclio_and_move_buttons_layout);

    auto buttons_layout = new QVBoxLayout;
    buttons_layout->addStretch(2);
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
    buttons_layout->addStretch(1);

    auto bottom_layout = new QHBoxLayout;
    bottom_layout->addLayout(distance_and_inclio_and_move_buttons_layout);
    bottom_layout->addStretch();
    bottom_layout->addLayout(buttons_layout);*/

    auto rootLayout = new QVBoxLayout;
    rootLayout->addLayout(camera_and_slice_layout);
    rootLayout->addLayout(distance_and_depth_layout);
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

void MainViewImpl::RunOnUiThread(const std::function<void()>& f)
{
    RunOnUiThread(this, f);
}

void MainViewImpl::SetDistance(double_t distance)
{
    //distance_viewport_->SetDistance(distance);

    std::uniform_real_distribution<> dis{0, 100};
    distance_viewport_->SetDistance(dis(gen));
}

void MainViewImpl::SetAngle(double_t angle)
{
    //inclio_viewport_->SetAngle(angle);

    std::uniform_real_distribution<> dis{0, 360};
    inclio_viewport_->SetAngle(dis(gen));
}

void MainViewImpl::SetDepth(int16_t depth)
{
    depth_label_->setText(QString::number(depth));

    slice_viewport_->SetDummySlice();
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
{}

std::string MainViewImpl::GetDestPath()
{
    return dest_path_edit_->text().toStdString();
}

void MainViewImpl::SetMoveForwardEnabled(bool enabled)
{
    move_forward_button_->setEnabled(enabled);
}

void MainViewImpl::SetMoveBackwardEnabled(bool enabled)
{
    move_backward_button_->setEnabled(enabled);
}

void MainViewImpl::SetRotateCcwEnabled(bool enabled)
{
    rotate_ccw_button_->setEnabled(enabled);
}

void MainViewImpl::SetRotateCwEnabled(bool enabled)
{
    rotate_cw_button_->setEnabled(enabled);
}

void MainViewImpl::SetSliceEnabled(bool enabled)
{
    slice_button_->setEnabled(enabled);
}

void MainViewImpl::SliceCompleted()
{
    slice_button_->setChecked(false);
}

void MainViewImpl::SetCamera1Enabled(bool enabled)
{
    camera1_button_->setEnabled(enabled);
}

void MainViewImpl::SetCamera2Enabled(bool enabled)
{
    camera2_button_->setEnabled(enabled);
}

void MainViewImpl::SetRecEnabled(bool enabled)
{
    rec_button_->setEnabled(enabled);
}

void MainViewImpl::SetPhotoEnabled(bool enabled)
{
    photo_button_->setEnabled(enabled);
}

void MainViewImpl::SetDestPathEnabled(bool enabled)
{
    dest_path_edit_->setEnabled(enabled);
}

void MainViewImpl::resizeEvent(QResizeEvent* event)
{
    (void)event;

    distance_viewport_->setMaximumSize(width() * 6 / 10, distance_viewport_height);
}

}}}
