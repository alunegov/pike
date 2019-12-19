#include <SliceWidget.h>

#include <algorithm>
#include <cmath>

#define _USE_MATH_DEFINES
#include <math.h>

#include <QtGui/QPainter>
#include <QtWidgets/QVBoxLayout>

namespace ros { namespace pike { namespace ui {

SliceWidget::SliceWidget()
{
    slice_view_ = new QLabel;
    slice_view_->setText("slice_view");
    //slice_view_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    // layout
    auto rootLayout = new QVBoxLayout;
    rootLayout->addWidget(slice_view_);

    setLayout(rootLayout);
}

void SliceWidget::SetSlice(const std::vector<double_t>& angles, const std::vector<int16_t>& depths)
{
    //
    const auto rect = slice_view_->rect();

    QPixmap pixmap{rect.width(), rect.height()};

    pixmap.fill(QColor{"transparent"});

    QPainter painter{&pixmap};

    painter.setRenderHint(QPainter::Antialiasing);
    //painter.setPen(QColor{"blue"});
    
    const auto max_depth = std::max(depths.begin(), depths.end());

    const auto r = std::min(rect.width(), rect.height()) / 2 - 5;
    const auto c = rect.center();

    //const double_t depth_coeff{(double_t)r / *max_depth};
    const double_t depth_coeff{(double_t)r / 1};

    QPolygon polygon;

    for (size_t i = 0; i < angles.size(); i++) {
        const auto x = c.x() + depths[i] * depth_coeff * std::cos(angles[i] * M_PI / 180);
        const auto y = c.y() + depths[i] * depth_coeff * std::sin(angles[i] * M_PI / 180);
        polygon.append(QPoint{(int)x, (int)y});
    }
    //polygon.append(polygon.at(0));

    painter.drawPolyline(polygon);

    slice_view_->setPixmap(pixmap);
}

void SliceWidget::SetDummySlice()
{
    std::vector<double_t> angles(360);
    for (size_t i = 0; i < angles.size(); i++) {
        angles[i] = i;
    }

    std::vector<int16_t> depths(360, 1);

    SetSlice(angles, depths);
}

}}}
