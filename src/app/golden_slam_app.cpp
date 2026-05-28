#include "app/golden_slam_app.hpp"
#include "core/slam_core.hpp"
#include "platform/ros_bridge.hpp"

#include <csignal>

static void sigHandle(int /*sig*/)
{
    rclcpp::shutdown();
}

GoldenSlamApp::GoldenSlamApp(int argc, char** argv)
    : argc_(argc), argv_(argv)
{
}

GoldenSlamApp::~GoldenSlamApp()
{
    if (rclcpp::ok())
        rclcpp::shutdown();
}

int GoldenSlamApp::run()
{
    signal(SIGINT, sigHandle);

    slamCore_  = std::make_unique<SlamCore>();
    rclcpp::init(argc_, argv_);
    rosBridge_ = std::make_shared<RosBridge>(slamCore_.get());

    rclcpp::spin(rosBridge_);
    rclcpp::shutdown();
    return 0;
}
