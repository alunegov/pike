#include <MainViewImpl.h>

#include <QtWidgets/QVBoxLayout>

namespace ros { namespace pike { namespace ui {

MainViewImpl::MainViewImpl(ros::pike::modules::MainPresenter* presenter) :
    presenter_{presenter}
{
    label_ = new QLabel;
    label_->setText("sadfsadfsadf");

    //
    auto verticalLayout = new QVBoxLayout;
    verticalLayout->addWidget(label_);

    setLayout(verticalLayout);

    //
    presenter_->SetView(this);

    presenter_->OnShow();
}

void MainViewImpl::SetDepth(int16_t value)
{
    label_->setText(QString::number(value));
}

}}}
