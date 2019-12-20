#include <CameraWidget.h>

#include <QtWidgets/QVBoxLayout>

namespace ros { namespace pike { namespace ui {

CameraWidget::CameraWidget()
{
    camera_view_ = new QLabel;
    camera_view_->setText("camera_view");
    camera_view_->setAlignment(Qt::AlignCenter);
    //camera_view_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);

    // layout
    auto rootLayout = new QVBoxLayout;
    rootLayout->addWidget(camera_view_);

    setLayout(rootLayout);
}

}}}
