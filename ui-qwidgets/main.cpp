#ifdef _MSC_VER
//#include <vld.h>
#endif

#include <cstdint>

#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QStatusBar>

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

    QApplication app{argc, argv};

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

    auto const statusBar = new QStatusBar{&win};
    statusBar->setObjectName("statusBar");
    win.setStatusBar(statusBar);

    win.show();

    auto conf = ros::pike::logic::ConfMapper::Load("pike.json");

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
        const uint16_t AdcCommonGnd{1 << 5};

        conf.inclinometer.x_channel |= AdcCommonGnd;
        conf.inclinometer.y_channel |= AdcCommonGnd;

        conf.odometer.a_channel |= AdcCommonGnd;
        conf.odometer.b_channel |= AdcCommonGnd;
    }

    // dc and devices
    //ros::dc::lcard::LCardDaq daq;
    ros::dc::dummy::DummyDaq daq;
    const auto daq_init_opt = daq.Init(conf.daq.slot);
    if (!daq_init_opt) {
        // TODO: log and cleanup
        QMessageBox::critical(nullptr, "pike", QString::fromStdString("daq init error: " + daq_init_opt.error().message()));

        return 1;
    }
    // плата "закроется" в pike, или при удалении daq
    const auto daq_ttlin_enable_opt = daq.TtlEnable(true);
    if (!daq_ttlin_enable_opt) {
        // TODO: log and cleanup
        QMessageBox::critical(nullptr, "pike", QString::fromStdString("daq ttl enable error: " + daq_ttlin_enable_opt.error().message()));

        return 1;
    }

    ros::devices::EnderImpl ender1{&daq, conf.ender1.pin};

    ros::devices::EnderImpl ender2{&daq, conf.ender2.pin};

    ros::devices::RotatorImpl rotator{&daq, conf.rotator.en_pin, conf.rotator.step_pin, conf.rotator.dir_pin,
            conf.rotator.mx_pin, conf.rotator.steps_per_msr, conf.rotator.steps_per_view};

    ros::devices::MoverImpl mover{&daq, conf.mover.pwm_pin, conf.mover.dir_pin};

    ros::devices::OdometerImpl odometer{conf.odometer.a_channel, conf.odometer.b_channel, conf.odometer.threshold,
            conf.odometer.distance_per_pulse};

    const auto trans_table = ros::devices::InclinometerImplTransTableMapper::Load(conf.inclinometer.trans_table_file);
    if (trans_table.size() < 2) {
        // TODO: log and cleanup
        QMessageBox::critical(nullptr, "pike", QString::fromStdString("empty trans table"));

        return 1;
    }

    ros::devices::InclinometerImpl inclinometer{conf.inclinometer.x_channel, conf.inclinometer.y_channel, trans_table};

    ce::ceSerial depthometer_transport{conf.depthometer.port_name, conf.depthometer.baud_rate, 8, 'N', 1};
    const auto open_res = depthometer_transport.Open();
    if (open_res != 0) {
        // TODO: log and cleanup
        QMessageBox::critical(nullptr, "pike", QString("COM open error: %1").arg(open_res));

#ifdef NDEBUG
        return 1;
#endif
    }
    // порт закроется в depthometer, или при удалении depthometer_transport (в конце main)

    ros::devices::CD22 depthometer{depthometer_transport};

    // logic/interactors/save-mappers
    ros::pike::logic::PikeImpl pike{&daq, &ender1, &ender2, &rotator, &mover, &odometer, &inclinometer, &depthometer};
    
    ros::pike::logic::OngoingReaderImpl ongoingReader{&pike, conf.daq.adc_rate};

    ros::pike::logic::SlicerImpl slicer{&pike};

    ros::pike::logic::SliceMsrMapperImpl sliceMsrMapper;

    ros::pike::logic::RemoteServerImpl remoteServer{conf.remote.port};

    // presenter and view
    ros::pike::modules::MainPresenterImpl mainPresenterImpl{&pike, &ongoingReader, &slicer, &sliceMsrMapper, &remoteServer};

    const auto setStatusMsgFunc = [&win](const std::string& msg) {
        assert(win.statusBar() != nullptr);
        win.statusBar()->showMessage(QString::fromStdString(msg));
    };

    auto const mainViewImpl = new ros::pike::ui::MainViewImpl{&mainPresenterImpl, conf.object_length, setStatusMsgFunc};
    // выставляем mainview как центральный виджет QMainWindow
    win.setCentralWidget(mainViewImpl);

    const auto app_res = QApplication::exec();

    // останавливаем всю обработку
    mainPresenterImpl.OnHide();

    // забираем управление mainview и сами удаляем его (иначе он удалится через DeleteLater)
    win.takeCentralWidget();
    delete mainViewImpl;

    return app_res;
}
