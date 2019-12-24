#pragma once

#include <cmath>

#include <QtGui/qevent.h>
#include <QtWidgets/QLabel>
#include <QtWidgets/QWidget>

namespace ros { namespace pike { namespace ui {

class DistanceWidget : public QWidget
{
    Q_OBJECT

public:
    explicit DistanceWidget(double_t max_distance);

    void SetDistance(double_t distance);

protected:
    void resizeEvent(QResizeEvent* event) override;

private:
    void update_view();

    QLabel* distance_view_{nullptr};
    
    double_t max_distance_{0};
    double_t distance_{0};
};

}}}
