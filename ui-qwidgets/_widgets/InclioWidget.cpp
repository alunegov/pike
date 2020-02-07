#include <InclioWidget.h>

#include <cmath>
#define _USE_MATH_DEFINES
#include <math.h>

#include <QTimer>
#include <QtGui/QPainter>
#include <QtWidgets/QVBoxLayout>

namespace ros { namespace pike { namespace ui {

InclioWidget::InclioWidget()
{
    angle_text_ = new QLabel;
    angle_text_->setText("angle_text");
    angle_text_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);

    angle_view_ = new QLabel;
    angle_view_->setText("angle_view");
    //angle_view_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

    pen1_.setStyle(Qt::SolidLine);
    pen1_.setColor(QColor{"blue"});
    pen2_.setStyle(Qt::DotLine);

    _anim.setDuration(AnimDuration);
    QObject::connect(&_anim, &QVariantAnimation::valueChanged, this, [this](const QVariant& value) {
        angle_ = value.toDouble();
        while (angle_ > 360) {
            angle_ -= 360;
        }
        update_view();
    });

    // layout
    auto rootLayout = new QVBoxLayout;
    rootLayout->addWidget(angle_text_);
    rootLayout->addWidget(angle_view_);

    setLayout(rootLayout);
}

void InclioWidget::SetAngle(double_t angle)
{
    _anim.stop();

    angle_text_->setText(QString{"%1°"}.arg(angle));

    //angle_ = angle;
    //update_view();
    const auto v = (std::abs(angle - angle_) <= 180)
        ? std::make_pair(angle_, angle)
        : std::make_pair((angle_ < angle) ? 360 + angle_ : angle_, (angle_ < angle) ? angle : 360 + angle);
    _anim.setStartValue(v.first);
    _anim.setEndValue(v.second);

    _anim.start();
}

void InclioWidget::resizeEvent(QResizeEvent* event)
{
    (void)event;

    update_view();
}

void InclioWidget::update_view()
{
    const auto rect = angle_view_->rect();

    QPixmap pixmap{rect.width(), rect.height()};

    pixmap.fill(QColor{"transparent"});

    QPainter painter{&pixmap};

    painter.setRenderHint(QPainter::Antialiasing);

    const auto r = std::min(rect.width(), rect.height()) / 2 - 5;
    const auto c = rect.center();

    painter.setPen(pen2_);
    painter.drawEllipse(c, r, r);

    const auto x = c.x() + r * std::cos(angle_ * M_PI / 180);
    const auto y = c.y() + r * std::sin(angle_ * M_PI / 180);
    painter.setPen(pen1_);
    painter.drawLine(c, QPoint{(int)x, (int)y});

    angle_view_->setPixmap(pixmap);
}

}}}
