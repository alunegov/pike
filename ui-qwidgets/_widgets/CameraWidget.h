#pragma once

#include <QtWidgets/QLabel>
#include <QtWidgets/QWidget>

namespace ros { namespace pike { namespace ui {

class CameraWidget : public QWidget
{
    Q_OBJECT

public:
    CameraWidget();

private:
    QLabel* camera_view_{nullptr};
};

}}}
