#include <MainViewImpl.h>

#include <cassert>

#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>

#include <Mover.h>
#include <Rotator.h>

namespace ros { namespace pike { namespace ui {

const size_t distance_viewport_height{50};
const size_t min_viewport_size{100};

MainViewImpl::MainViewImpl(ros::pike::modules::MainPresenter* presenter, double_t object_length, const SetStatusMsgFunc& setStatusMsgFunc) :
    presenter_{presenter},
    _setStatusMsgFunc{setStatusMsgFunc}
{
    assert(presenter != nullptr);

    camera_viewport_ = new CameraWidget;
    camera_viewport_->setMinimumSize(min_viewport_size, min_viewport_size);
    camera_viewport_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);

    distance_viewport_ = new DistanceWidget{object_length};
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
    move_forward_button_->setText("Вперёд");
    move_forward_button_->setObjectName("motion");
    QObject::connect(move_forward_button_, &QPushButton::pressed, this, [=]() {
        presenter_->StartMovement(ros::devices::MoverDirection::Forward);
    });
    QObject::connect(move_forward_button_, &QPushButton::released, this, [=]() {
        move_forward_button_->setCheckable(false);
        presenter_->StopMovement();
    });

    move_backward_button_ = new QPushButton;
    move_backward_button_->setText("Назад");
    move_backward_button_->setObjectName("motion");
    QObject::connect(move_backward_button_, &QPushButton::pressed, this, [=]() {
        presenter_->StartMovement(ros::devices::MoverDirection::Backward);
    });
    QObject::connect(move_backward_button_, &QPushButton::released, this, [=]() {
        move_backward_button_->setCheckable(false);
        presenter_->StopMovement();
    });

    rotate_ccw_button_ = new QPushButton;
    rotate_ccw_button_->setText("Вращ.\nвлево");
    rotate_ccw_button_->setObjectName("motion");
    QObject::connect(rotate_ccw_button_, &QPushButton::pressed, this, [=]() {
        presenter_->StartRotation(ros::devices::RotatorDirection::CCW);
    });
    QObject::connect(rotate_ccw_button_, &QPushButton::released, this, [=]() {
        rotate_ccw_button_->setCheckable(false);
        presenter_->StopRotation();
    });

    rotate_cw_button_ = new QPushButton;
    rotate_cw_button_->setText("Вращ.\nвправо");
    rotate_cw_button_->setObjectName("motion");
    QObject::connect(rotate_cw_button_, &QPushButton::pressed, this, [=]() {
        presenter_->StartRotation(ros::devices::RotatorDirection::CW);
    });
    QObject::connect(rotate_cw_button_, &QPushButton::released, this, [=]() {
        rotate_cw_button_->setCheckable(false);
        presenter_->StopRotation();
    });

    slice_button_ = new QPushButton;
    slice_button_->setText("Запись/отобр. диаметра");
    slice_button_->setCheckable(true);
    slice_button_->setObjectName("record");
    QObject::connect(slice_button_, &QPushButton::toggled, this, [=](bool checked) {
        if (checked) {
            presenter_->StartSlice(std::move(dest_path_edit_->text().toStdString()));
        } else {
            presenter_->StopSlice();
        }
    });

    camera1_button_ = new QPushButton;
    camera1_button_->setText("Камера 1");
    QObject::connect(camera1_button_, &QPushButton::clicked, this, [=]() {
        presenter_->Camera1Clicked();
    });

    camera2_button_ = new QPushButton;
    camera2_button_->setText("Камера 2");
    QObject::connect(camera2_button_, &QPushButton::clicked, this, [=]() {
        presenter_->Camera2Clicked();
    });

    rec_button_ = new QPushButton;
    rec_button_->setText("Камера/Запись");
    rec_button_->setCheckable(true);
    rec_button_->setObjectName("record");
    QObject::connect(rec_button_, &QPushButton::toggled, this, [=](bool checked) {
        if (checked) {
            presenter_->StartRec(std::move(dest_path_edit_->text().toStdString()));
        } else {
            presenter_->StopRec();
        }
    });

    photo_button_ = new QPushButton;
    photo_button_->setText("Камера/Фото");
    QObject::connect(photo_button_, &QPushButton::clicked, this, [=]() {
        presenter_->PhotoClicked(std::move(dest_path_edit_->text().toStdString()));
    });

    dest_path_edit_ = new QLineEdit;
    dest_path_edit_->setText("путь для записи");

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
    }
}

void MainViewImpl::RunOnUiThread(const std::function<void()>& f)
{
    RunOnUiThread(this, f);
}

void MainViewImpl::SetStatusMsg(const std::string& msg)
{
    _setStatusMsgFunc(msg);
}

void MainViewImpl::SetDistance(double_t distance)
{
    //distance_viewport_->SetDistance(distance);

    // debug
    std::uniform_real_distribution<> dis{0, 100};
    distance_viewport_->SetDistance(dis(gen));

    slice_viewport_->SetDummySlice();
}

void MainViewImpl::SetAngle(double_t angle)
{
    //inclio_viewport_->SetAngle(angle);

    // debug
    std::uniform_real_distribution<> dis{0, 360};
    inclio_viewport_->SetAngle(dis(gen));
}

void MainViewImpl::SetDepth(int16_t depth)
{
    depth_label_->setText(QString{"%1 мкм"}.arg(depth));
}

void MainViewImpl::UpdateSliceDepth(double_t angle, int16_t depth)
{
    depth_label_->setText(QString{"%1° - %2 мкм"}.arg(angle).arg(depth));

    // TODO: постепенное отображение на slice_viewport_ по мере измерения
}

void MainViewImpl::SetSliceMsr(const std::vector<double_t>& angles, const std::vector<int16_t>& depths)
{
    slice_viewport_->SetSlice(angles, depths);
}

void MainViewImpl::SetEnders(bool ender1, bool ender2)
{
    ender1_label_->setText(QString::number(ender1 ? 1 : 0));
    ender2_label_->setText(QString::number(ender2 ? 1 : 0));
}

void MainViewImpl::SetAdcChannels(const std::vector<uint16_t>& channels, const int16_t* values, size_t values_count,
        double_t adc_to_volt)
{}

void MainViewImpl::SetMoveForwardEnabled(bool enabled)
{
    move_forward_button_->setEnabled(enabled);
}

void MainViewImpl::SetMoveForward(bool checked)
{
    move_forward_button_->setCheckable(checked);
    move_forward_button_->setChecked(checked);
    move_forward_button_->update();
}

void MainViewImpl::SetMoveBackwardEnabled(bool enabled)
{
    move_backward_button_->setEnabled(enabled);
}

void MainViewImpl::SetMoveBackward(bool checked)
{
    move_backward_button_->setCheckable(checked);
    move_backward_button_->setChecked(checked);
    move_backward_button_->update();
}

void MainViewImpl::SetRotateCcwEnabled(bool enabled)
{
    rotate_ccw_button_->setEnabled(enabled);
}

void MainViewImpl::SetRotateCcw(bool checked)
{
    rotate_ccw_button_->setCheckable(checked);
    rotate_ccw_button_->setChecked(checked);
    rotate_ccw_button_->update();
}

void MainViewImpl::SetRotateCwEnabled(bool enabled)
{
    rotate_cw_button_->setEnabled(enabled);
}

void MainViewImpl::SetRotateCw(bool checked)
{
    rotate_cw_button_->setCheckable(checked);
    rotate_cw_button_->setChecked(checked);
    rotate_cw_button_->update();
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
