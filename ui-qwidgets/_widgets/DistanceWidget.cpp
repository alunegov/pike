#include <DistanceWidget.h>

#include <cassert>

#include <QtGui/QPainter>
#include <QtWidgets/QVBoxLayout>

namespace ros { namespace pike { namespace ui {

DistanceWidget::DistanceWidget(double_t max_distance) :
    max_distance_{max_distance}
{
    assert(max_distance > 0);

    distance_view_ = new QLabel;
    distance_view_->setText("distance");
    distance_view_->setAlignment(Qt::AlignCenter);
    //distance_view_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);

    // layout
    auto rootLayout = new QVBoxLayout;
    rootLayout->addWidget(distance_view_);

    setLayout(rootLayout);
}

void DistanceWidget::SetDistance(double_t distance)
{
    distance_ = distance;

    distance_view_->setText(QString{"%1 мм"}.arg(distance_));
    update_view();
}

void DistanceWidget::resizeEvent(QResizeEvent* event)
{
    (void)event;

    update_view();
}

void DistanceWidget::update_view()
{
    assert(max_distance_ > 0);

    const auto rect = distance_view_->rect();

    QPixmap pixmap{rect.width(), rect.height()};

    //pixmap.fill(QColor{"transparent"});

    QPainter painter{&pixmap};

    // пока выводим прямоугольники и текст Antialiasing не особо и нужен
    //painter.setRenderHint(QPainter::Antialiasing);
    //painter.setPen(QColor{"blue"});

    const auto w = rect.width();
    const auto rw = rect.height();

    const auto distance_coeff = w / max_distance_;

    //QImage img{"devicesetups.svg"};
    //img.rec
    //painter.drawImage(rect, img);

    painter.drawText(distance_ * distance_coeff, rect.height(), QString::number(distance_));

    painter.drawRect(distance_ * distance_coeff, 0, rw, rect.height());

    distance_view_->setPixmap(pixmap);
}

}}}
