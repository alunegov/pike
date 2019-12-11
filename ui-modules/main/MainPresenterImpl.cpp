#include <MainPresenterImpl.h>

#include <chrono>
#include <future>
#include <thread>

namespace ros { namespace pike { namespace modules {

MainPresenterImpl::~MainPresenterImpl()
{
    //delete pike_;
}

void MainPresenterImpl::SetView(ros::pike::modules::MainView* view)
{
    view_ = view;
}

void MainPresenterImpl::OnShow()
{
    std::thread{[&]() {
        while (true) {
            const auto depth = pike_->depthometer()->Read();
            view_->SetDepth(depth);

            std::this_thread::sleep_for(std::chrono::milliseconds{1000});
        }
    }}.detach();
}

}}}
