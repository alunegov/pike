#include <QtTest/QtTest>
#include <QtWidgets/QtWidgets>

#include <qtest/fakeit.hpp>

#include <MainPresenter.h>
#include <MainViewImpl.h>

class MainViewImplTest: public QObject
{
    Q_OBJECT

private slots:
    void init();

    void testSetDistance();

    void testSetAngle();

private:
    fakeit::Mock<ros::pike::modules::MainPresenter> presenter_mock;
};

void MainViewImplTest::init()
{
    presenter_mock.Reset();
}

void MainViewImplTest::testSetDistance()
{
    fakeit::Fake(Method(presenter_mock, SetView));
    fakeit::Fake(Method(presenter_mock, OnShow));

    auto& p = presenter_mock.get();

    ros::pike::ui::MainViewImpl sut{&p, 110};

    sut.SetDistance(13.0);
    //QVERIFY(sut.distance_label_->text() == "13");
}

void MainViewImplTest::testSetAngle()
{
    fakeit::Fake(Method(presenter_mock, SetView));
    fakeit::Fake(Method(presenter_mock, OnShow));

    auto& p = presenter_mock.get();

    ros::pike::ui::MainViewImpl sut{&p, 110};

    sut.SetAngle(13.0);
    //QVERIFY(sut.angle_label_->text() == "13");
}

QTEST_MAIN(MainViewImplTest)

#include "MainViewImplTest.moc"
