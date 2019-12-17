#ifdef _MSC_VER
//#include <vld.h>
#endif

#include <cstdint>

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

#include <ConfMapper.h>
#include <OngoingReaderImpl.h>
#include <SlicerImpl.h>

#include <MainPresenterImpl.h>
#include <MainViewImpl.h>

int main(int argc, char** argv)
{
    const uint16_t AdcCommonGnd{1 << 5};

    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

    QApplication app(argc, argv);

    QMainWindow win;
    win.show();

    auto conf = ros::pike::logic::ConfMapper::Load("conf.json");

    // адаптаци€ настроек под Ћ- ард (нумераци€ каналов/пинов с нул€ и флаг общей земли)
    conf.ender1.pin -= 1;
    conf.ender2.pin -= 1;
    conf.inclinometer.x_channel -= 1;
    if (conf.daq.common_gnd) {
        conf.inclinometer.x_channel |= AdcCommonGnd;
    }
    conf.inclinometer.y_channel -= 1;
    if (conf.daq.common_gnd) {
        conf.inclinometer.y_channel |= AdcCommonGnd;
    }
    conf.mover.pwm_pin -= 1;
    conf.mover.dir_pin -= 1;
    conf.odometer.a_channel -= 1;
    if (conf.daq.common_gnd) {
        conf.odometer.a_channel |= AdcCommonGnd;
    }
    conf.odometer.b_channel -= 1;
    if (conf.daq.common_gnd) {
        conf.odometer.b_channel |= AdcCommonGnd;
    }
    conf.rotator.en_pin -= 1;
    conf.rotator.step_pin -= 1;
    conf.rotator.dir_pin -= 1;
    conf.rotator.mx_pin -= 1;

    // dc and devices
    auto daq = new ros::dc::lcard::LCardDevice;
    daq->Init(conf.daq.slot);
    daq->TtlEnable(true);
    // плата "закрываетс€" в pike

    auto ender1 = new ros::devices::EnderImpl{daq, conf.ender1.pin};

    auto ender2 = new ros::devices::EnderImpl{daq, conf.ender2.pin};

    auto rotator = new ros::devices::RotatorImpl{daq, conf.rotator.en_pin, conf.rotator.step_pin, conf.rotator.dir_pin,
            conf.rotator.mx_pin};

    auto mover = new ros::devices::MoverImpl{daq, conf.mover.pwm_pin, conf.mover.dir_pin};

    auto odometer = new ros::devices::OdometerImpl{conf.odometer.a_channel, conf.odometer.b_channel,
            conf.odometer.threshold, conf.odometer.distance_per_pulse};

    auto trans_table = ros::devices::InclinometerImplTransTableMapper::Load(conf.inclinometer.trans_table_file);

    auto inclinometer = new ros::devices::InclinometerImpl{conf.inclinometer.x_channel, conf.inclinometer.y_channel,
            trans_table};

    ce::ceSerial depthometer_transport{conf.depthometer.port_name, conf.depthometer.baud_rate, 8, 'N', 1};
    depthometer_transport.Open();
    // порт закроетс€ в depthometer, или автоматически при удалении depthometer_transport (в конце main)

    auto depthometer = new ros::devices::CD22{depthometer_transport};

    auto pike = new ros::devices::PikeImpl{daq, ender1, ender2, rotator, mover, odometer, inclinometer, depthometer};

    // logic/interactors
    auto ongoingReader = new ros::pike::logic::OngoingReaderImpl{pike, conf.daq.adc_rate};

    auto slicer = new ros::pike::logic::SlicerImpl{pike};

    // presenter and view
    auto mainPresenterImpl = new ros::pike::modules::MainPresenterImpl{pike, ongoingReader, slicer};

    auto mainViewImpl = new ros::pike::ui::MainViewImpl{mainPresenterImpl};

    // set view as center widget
    win.setCentralWidget(mainViewImpl);

    return QApplication::exec();
}
