#pragma once

#include <cmath>

#include <QtWidgets/QLabel>
#include <QtGui/QPen>
#include <QtWidgets/QWidget>

namespace ros { namespace pike { namespace ui {

class InclioWidget : public QWidget
{
    Q_OBJECT

public:
    InclioWidget();

    void SetAngle(double_t angle);

private:
    QLabel* angle_text_{nullptr};
    QLabel* angle_view_{nullptr};

    QPen pen1_;
    QPen pen2_;
};

}}}
