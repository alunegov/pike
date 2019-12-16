#ifdef _MSC_VER
//#include <vld.h>
#endif

#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>

#include <ceSerial.h>

#include <LCardDevice.h>

#include <CD22.h>
#include <EnderImpl.h>
#include <InclinometerImpl.h>
#include <InclinometerImplTransTableMapper.h>
#include <MoverImpl.h>
#include <OdometerImpl.h>
#include <PikeImpl.h>
#include <RotatorImpl.h>

#include <OngoingReaderImpl.h>
#include <SlicerImpl.h>

#include <MainPresenterImpl.h>
#include <MainViewImpl.h>

int main(int argc, char** argv)
{
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

    QApplication app(argc, argv);

    QMainWindow win;
    win.show();

    auto daq = new ros::dc::lcard::LCardDevice;
    daq->Init(0u);
    daq->TtlEnable(true);

    auto ender1 = new ros::devices::EnderImpl{daq, 3u - 1u};

    auto ender2 = new ros::devices::EnderImpl{daq, 4u - 1u};

    auto rotator = new ros::devices::RotatorImpl{daq, 3u - 1u, 5u - 1u, 4u - 1u, 6u - 1u};

    auto mover = new ros::devices::MoverImpl{daq, 1u - 1u, 2u - 1u};

    auto odometer = new ros::devices::OdometerImpl{(5u - 1u) | 32u, (6u - 1u) | 32u};

    auto trans_table = ros::devices::InclinometerImplTransTableMapper::Load("inclinometer.tbl");

    auto inclinometer = new ros::devices::InclinometerImpl{(1u - 1u) | 32u, (2u - 1u) | 32u, trans_table};

    ce::ceSerial depthometer_transport{R"(\\.\COM43)", 115200, 8, 'N', 1};
    depthometer_transport.Open();

    auto depthometer = new ros::devices::CD22{depthometer_transport};

    auto pike = new ros::devices::PikeImpl{daq, ender1, ender2, rotator, mover, odometer, inclinometer, depthometer};

    auto ongoingReader = new ros::pike::logic::OngoingReaderImpl{pike};

    auto slicer = new ros::pike::logic::SlicerImpl{pike};

    auto mainPresenterImpl = new ros::pike::modules::MainPresenterImpl{pike, ongoingReader, slicer};

    auto mainViewImpl = new ros::pike::ui::MainViewImpl{mainPresenterImpl};

    win.setCentralWidget(mainViewImpl);

    return QApplication::exec();
}
