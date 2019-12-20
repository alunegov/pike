#include <SliceWidget.h>

#include <algorithm>
#include <cmath>
#define _USE_MATH_DEFINES
#include <math.h>
#include <random>

#include <QtGui/QPainter>
#include <QtWidgets/QVBoxLayout>

#include <RosMath.h>

namespace ros { namespace pike { namespace ui {

SliceWidget::SliceWidget()
{
    slice_view_ = new QLabel;
    slice_view_->setText("slice_view");
    //slice_view_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);

    pen1_.setStyle(Qt::SolidLine);
    pen2_.setStyle(Qt::DotLine);
    pen3_.setStyle(Qt::DashLine);

    // layout
    auto rootLayout = new QVBoxLayout;
    rootLayout->addWidget(slice_view_);

    setLayout(rootLayout);
}

void SliceWidget::SetSlice(const std::vector<double_t>& angles, const std::vector<int16_t>& depths)
{
    angles_.assign(angles.begin(), angles.end());
    depths_.assign(depths.begin(), depths.end());

    update_view();
}

void SliceWidget::SetDummySlice()
{
    const double_t max_depth{5000};

    // чтобы разрешение было повыше
    const size_t points_per_degree{2};

    //
    angles_.resize(360 * points_per_degree);

    for (size_t i = 0; i < angles_.size(); i++) {
        angles_[i] = (double_t)i / points_per_degree;
    }

    //
    depths_.resize(360 * points_per_degree);

    std::random_device rd;
    std::mt19937 gen{rd()};
    std::uniform_real_distribution<> dis{-max_depth, max_depth};

    for (size_t i = 0; i < depths_.size(); i++) {
        depths_[i] = dis(gen);
    }

    update_view();
}

void SliceWidget::resizeEvent(QResizeEvent* event)
{
    (void)event;

    update_view();
}

void SliceWidget::update_view()
{
    const auto rect = slice_view_->rect();

    QPixmap pixmap{rect.width(), rect.height()};

    pixmap.fill(QColor{"transparent"});

    QPainter painter{&pixmap};

    painter.setRenderHint(QPainter::Antialiasing);
    //painter.setPen(QColor{"blue"});

    // r1 + r2 + r3 = R
    const auto R = std::min(rect.width(), rect.height()) / 2;
    const auto r1 = R * 7 / 10;  // 70% from R
    const auto r3 = 5;
    const auto r2 = R - r1 - r3;

    const auto dr = r1 + r2 / 2;

    const auto c = rect.center();

    /*painter.setPen(pen2_);
    painter.drawEllipse(c, r1, r1);  // линия мин
    painter.drawEllipse(c, dr, dr);  // линия среднего
    painter.drawEllipse(c, r1 + r2, r1 + r2);  // линия макс*/

    const auto minmax_depth = std::minmax_element(depths_.begin(), depths_.end());
    const auto mean_depth = Mean_AdcRaw(depths_.data(), (uint32_t)depths_.size());

    double_t depth_coeff{1};
    if (*minmax_depth.second != *minmax_depth.first) {
        depth_coeff = (double_t)r2 / (*minmax_depth.second - *minmax_depth.first);
    } else {
        depth_coeff = 0.0;
    }

    // линия нуля
    const auto rr = dr + (0 - mean_depth) * depth_coeff;
    painter.setPen(pen3_);
    painter.drawEllipse(c, (int)rr, (int)rr);

    QPolygon polygon;
    for (size_t i = 0; i < angles_.size(); i++) {
        const auto x = c.x() + (dr + (depths_[i] - mean_depth) * depth_coeff) * std::cos(angles_[i] * M_PI / 180);
        const auto y = c.y() + (dr + (depths_[i] - mean_depth) * depth_coeff) * std::sin(angles_[i] * M_PI / 180);
        polygon.append(QPoint{(int)x, (int)y});
    }
    
    painter.setPen(pen1_);
    painter.drawPolyline(polygon);

    slice_view_->setPixmap(pixmap);
}

}}}
