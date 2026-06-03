#include "app/golden_slam_app.hpp"
#include "core/slam_core.hpp"
#include "platform/ros_bridge.hpp"

GoldenSlamApp::GoldenSlamApp(int argc, char** argv)
    : argc_(argc) , argv_(argv)
{
    
}

GoldenSlamApp::~GoldenSlamApp()
{
    RosBridge::rosShutdown();
}

int GoldenSlamApp::run() 
{
    slamCore_ = std::make_unique<SlamCore>();
    RosBridge::rosInit(argc_,argv_);
    rosBridge_ = std::make_shared<RosBridge>(slamCore_.get());
    rosBridge_->spin();

    return 0;
}