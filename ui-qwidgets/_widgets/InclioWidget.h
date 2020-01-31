#pragma once

#include <cmath>

#include <QtCore/QVariantAnimation>
#include <QtGui/qevent.h>
#include <QtGui/QPen>
#include <QtWidgets/QLabel>
#include <QtWidgets/QWidget>

namespace ros { namespace pike { namespace ui {

class InclioWidget : public QWidget
{
    Q_OBJECT

public:
    InclioWidget();

    void SetAngle(double_t angle);

protected:
    void resizeEvent(QResizeEvent* event) override;

private:
    const int AnimDuration{250};

    void update_view();

    QLabel* angle_text_{nullptr};
    QLabel* angle_view_{nullptr};

    QPen pen1_;
    QPen pen2_;

    double_t angle_{0};

    QVariantAnimation _anim;
};

}}}
