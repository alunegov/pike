#ifdef _MSC_VER
#include <vld.h>
#endif

#include <cstdint>

#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>

#include <ceSerial.h>

#include <DummyDaq.h>
#include <LCardDaq.h>

#include <CD22.h>
#include <EnderImpl.h>
#include <InclinometerImpl.h>
#include <InclinometerImplTransTableMapper.h>
#include <MoverImpl.h>
#include <OdometerImpl.h>
#include <RotatorImpl.h>

#include <ConfMapper.h>
#include <OngoingReaderImpl.h>
#include <PikeImpl.h>
#include <RemoteServerImpl.h>
#include <SliceMsrMapperImpl.h>
#include <SlicerImpl.h>

#include <MainPresenterImpl.h>
#include <MainViewImpl.h>

int main(int argc, char** argv)
{
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

    QApplication app(argc, argv);
    // стиль для всего приложения
    const auto style = QString{R"(
        QPushButton {
            min-height: %1px;
            font-size: %2px;
        }

        QPushButton#record:checked, QPushButton#motion:pressed, QPushButton#motion:checked {
            background-color: red;
        }
    )"}.arg(24).arg(14);
    app.setStyleSheet(style);

    QMainWindow win;
    win.setMinimumSize(640, 480);
    win.show();

    auto conf = ros::pike::logic::ConfMapper::Load("conf.json");

    // адаптация настроек под Л-Кард (нумерация каналов/пинов с нуля и флаг общей земли)
    conf.ender1.pin -= 1;
    conf.ender2.pin -= 1;
    conf.inclinometer.x_channel -= 1;
    conf.inclinometer.y_channel -= 1;
    conf.mover.pwm_pin -= 1;
    conf.mover.dir_pin -= 1;
    conf.odometer.a_channel -= 1;
    conf.odometer.b_channel -= 1;
    conf.rotator.en_pin -= 1;
    conf.rotator.step_pin -= 1;
    conf.rotator.dir_pin -= 1;
    conf.rotator.mx_pin -= 1;
    if (conf.daq.common_gnd) {
        constexpr uint16_t AdcCommonGnd{1 << 5};

        conf.inclinometer.x_channel |= AdcCommonGnd;
        conf.inclinometer.y_channel |= AdcCommonGnd;

        conf.odometer.a_channel |= AdcCommonGnd;
        conf.odometer.b_channel |= AdcCommonGnd;
    }

    // dc and devices
    //auto daq = new ros::dc::lcard::LCardDaq;
    auto daq = new ros::dc::dummy::DummyDaq;
    const auto daq_init_opt = daq->Init(conf.daq.slot);
    if (!daq_init_opt) {
        // TODO: log and cleanup
        return 1;
    }
    const auto daq_ttlin_enable_opt = daq->TtlEnable(true);
    if (!daq_ttlin_enable_opt) {
        // TODO: log and cleanup
        return 1;
    }
    // плата "закрывается" в pike

    auto ender1 = new ros::devices::EnderImpl{daq, conf.ender1.pin};

    auto ender2 = new ros::devices::EnderImpl{daq, conf.ender2.pin};

    auto rotator = new ros::devices::RotatorImpl{daq, conf.rotator.en_pin, conf.rotator.step_pin, conf.rotator.dir_pin,
            conf.rotator.mx_pin, conf.rotator.steps_per_msr, conf.rotator.steps_per_view};

    auto mover = new ros::devices::MoverImpl{daq, conf.mover.pwm_pin, conf.mover.dir_pin};

    auto odometer = new ros::devices::OdometerImpl{conf.odometer.a_channel, conf.odometer.b_channel,
            conf.odometer.threshold, conf.odometer.distance_per_pulse};

    auto trans_table = ros::devices::InclinometerImplTransTableMapper::Load(conf.inclinometer.trans_table_file);

    auto inclinometer = new ros::devices::InclinometerImpl{conf.inclinometer.x_channel, conf.inclinometer.y_channel,
            trans_table};

    ce::ceSerial depthometer_transport{conf.depthometer.port_name, conf.depthometer.baud_rate, 8, 'N', 1};
    const auto open_res = depthometer_transport.Open();
    if (open_res != 0) {
        // TODO: log and cleanup
        //return 1;
    }
    // порт закроется в depthometer, или автоматически при удалении depthometer_transport (в конце main)

    auto depthometer = new ros::devices::CD22{depthometer_transport};

    auto pike = new ros::pike::logic::PikeImpl{daq, ender1, ender2, rotator, mover, odometer, inclinometer, depthometer};

    // logic/interactors/save-mappers
    auto ongoingReader = new ros::pike::logic::OngoingReaderImpl{pike, conf.daq.adc_rate};

    auto slicer = new ros::pike::logic::SlicerImpl{pike};

    auto sliceMsrMapper = new ros::pike::logic::SliceMsrMapperImpl;

    auto remoteServer = new ros::pike::logic::RemoteServerImpl{45454};  // TODO: port from conf

    // presenter and view
    auto mainPresenterImpl = new ros::pike::modules::MainPresenterImpl{pike, ongoingReader, slicer, sliceMsrMapper,
            remoteServer};

    auto mainViewImpl = new ros::pike::ui::MainViewImpl{mainPresenterImpl, conf.object_length};
    // выставляем mainview как центральный виджет QMainWindow
    win.setCentralWidget(mainViewImpl);

    const auto app_res = QApplication::exec();

    // забираем управление mainview и сами удаляем его (иначе он удалится через DeleteLater)
    win.takeCentralWidget();
    delete mainViewImpl;

    delete mainPresenterImpl;

    delete ongoingReader;
    delete slicer;
    delete sliceMsrMapper;
    delete remoteServer;

    // удаляемые выше зависят от pike
    delete pike;

    delete ender1;
    delete ender2;
    delete rotator;
    delete mover;
    delete odometer;
    delete inclinometer;
    delete depthometer;

    // удаляемые выше зависят от daq
    delete daq;

    return app_res;
}
