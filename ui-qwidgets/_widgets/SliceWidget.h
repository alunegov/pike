#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include <QtGui/qevent.h>
#include <QtGui/QPen>
#include <QtWidgets/QLabel>
#include <QtWidgets/QWidget>

namespace ros { namespace pike { namespace ui {

class SliceWidget : public QWidget
{
    Q_OBJECT

public:
    SliceWidget();

    void SetSlice(const std::vector<double_t>& angles, const std::vector<int16_t>& depths);

    void SetDummySlice();

protected:
    void resizeEvent(QResizeEvent* event) override;

private:
    void update_view();

    QLabel* slice_view_{nullptr};

    QPen pen1_;
    QPen pen2_;
    QPen pen3_;

    std::vector<double_t> angles_;
    std::vector<int16_t> depths_;
};

}}}
