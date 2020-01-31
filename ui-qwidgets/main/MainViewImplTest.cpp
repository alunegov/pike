#include <future>
#include <thread>

#include <QtTest/QTest>
#include <QtWidgets/QApplication>

#include <qtest/fakeit.hpp>

#include <MainPresenter.h>
#include <MainViewImpl.h>

class MainViewImplTest: public QObject
{
    Q_OBJECT

private slots:
    void init();

    void testRunOnUiThread_UiThread();

    void testRunOnUiThread_WorkerThread();

    void testSetStatusMsg();

    void testSetDistance();

    void testSetAngle();

    void testSetDepth();

    void testUpdateSliceDepth();

    void testSetSliceMsr();

    void testSetEnders();

private:
    const double_t ObjectLength{110};

    void DummySetStatusMsg(const std::string& msg)
    {}

    fakeit::Mock<ros::pike::modules::MainPresenter> presenter_mock;
};

using std::placeholders::_1;

void MainViewImplTest::init()
{
    presenter_mock.Reset();
}

void MainViewImplTest::testRunOnUiThread_UiThread()
{
    fakeit::Fake(Method(presenter_mock, SetView));
    fakeit::Fake(Method(presenter_mock, OnShow));

    auto& p = presenter_mock.get();

    ros::pike::ui::MainViewImpl sut{&p, ObjectLength, std::bind(&MainViewImplTest::DummySetStatusMsg, this, _1)};

    const auto caller_id = std::this_thread::get_id();
    bool test{false};

    sut.RunOnUiThread([caller_id, &test]() {
        test = true;
        QVERIFY(std::this_thread::get_id() == caller_id);
    });

    QVERIFY(test);
}

void MainViewImplTest::testRunOnUiThread_WorkerThread()
{
    fakeit::Fake(Method(presenter_mock, SetView));
    fakeit::Fake(Method(presenter_mock, OnShow));

    auto& p = presenter_mock.get();

    ros::pike::ui::MainViewImpl sut{&p, ObjectLength, std::bind(&MainViewImplTest::DummySetStatusMsg, this, _1)};

    const auto caller_id = std::this_thread::get_id();
    bool test{false};

    const auto f = std::async(std::launch::async, [&sut, caller_id, &test]() {
        const auto tmp_id = std::this_thread::get_id();

        sut.RunOnUiThread([caller_id, tmp_id, &test]() {
            test = true;
            QVERIFY(std::this_thread::get_id() == caller_id);
            QVERIFY(std::this_thread::get_id() != tmp_id);
        });
    });

    f.wait();
    QApplication::processEvents();

    QVERIFY(test);
}

void MainViewImplTest::testSetStatusMsg()
{
    fakeit::Fake(Method(presenter_mock, SetView));
    fakeit::Fake(Method(presenter_mock, OnShow));

    auto& p = presenter_mock.get();

    ros::pike::ui::MainViewImpl sut{&p, ObjectLength, [](const std::string& msg) {
        QVERIFY(msg == "test");
    }};

    sut.SetStatusMsg("test");
}

void MainViewImplTest::testSetDistance()
{
    fakeit::Fake(Method(presenter_mock, SetView));
    fakeit::Fake(Method(presenter_mock, OnShow));

    auto& p = presenter_mock.get();

    ros::pike::ui::MainViewImpl sut{&p, ObjectLength, std::bind(&MainViewImplTest::DummySetStatusMsg, this, _1)};

    sut.SetDistance(13.0);
    //QVERIFY(sut.distance_label_->text() == "13");
}

void MainViewImplTest::testSetAngle()
{
    fakeit::Fake(Method(presenter_mock, SetView));
    fakeit::Fake(Method(presenter_mock, OnShow));

    auto& p = presenter_mock.get();

    ros::pike::ui::MainViewImpl sut{&p, ObjectLength, std::bind(&MainViewImplTest::DummySetStatusMsg, this, _1)};

    sut.SetAngle(13.0);
    //QVERIFY(sut.angle_label_->text() == "13");
}

void MainViewImplTest::testSetDepth()
{
    fakeit::Fake(Method(presenter_mock, SetView));
    fakeit::Fake(Method(presenter_mock, OnShow));

    auto& p = presenter_mock.get();

    ros::pike::ui::MainViewImpl sut{&p, ObjectLength, std::bind(&MainViewImplTest::DummySetStatusMsg, this, _1)};

    sut.SetDepth(13);
    QVERIFY(sut.depth_label_->text().contains("13"));
}

void MainViewImplTest::testUpdateSliceDepth()
{
    fakeit::Fake(Method(presenter_mock, SetView));
    fakeit::Fake(Method(presenter_mock, OnShow));

    auto& p = presenter_mock.get();

    ros::pike::ui::MainViewImpl sut{&p, ObjectLength, std::bind(&MainViewImplTest::DummySetStatusMsg, this, _1)};

    sut.UpdateSliceDepth(13.1, 14);
    const auto t = sut.depth_label_->text();
    QVERIFY(t.contains("13.1") && t.contains("14"));
}

void MainViewImplTest::testSetSliceMsr()
{
    fakeit::Fake(Method(presenter_mock, SetView));
    fakeit::Fake(Method(presenter_mock, OnShow));

    auto& p = presenter_mock.get();

    ros::pike::ui::MainViewImpl sut{&p, ObjectLength, std::bind(&MainViewImplTest::DummySetStatusMsg, this, _1)};

    sut.SetSliceMsr({0, 1}, {10, 11});
}

void MainViewImplTest::testSetEnders()
{
    fakeit::Fake(Method(presenter_mock, SetView));
    fakeit::Fake(Method(presenter_mock, OnShow));

    auto& p = presenter_mock.get();

    ros::pike::ui::MainViewImpl sut{&p, ObjectLength, std::bind(&MainViewImplTest::DummySetStatusMsg, this, _1)};

    sut.SetEnders(true, false);
    QVERIFY((sut.ender1_label_->text() == "1") && (sut.ender2_label_->text() == "0"));

    sut.SetEnders(false, true);
    QVERIFY((sut.ender1_label_->text() == "0") && (sut.ender2_label_->text() == "1"));
}

QTEST_MAIN(MainViewImplTest)

#include "MainViewImplTest.moc"
