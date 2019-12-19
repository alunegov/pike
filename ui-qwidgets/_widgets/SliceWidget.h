#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

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

private:
    QLabel* slice_view_{nullptr};
};

}}}
