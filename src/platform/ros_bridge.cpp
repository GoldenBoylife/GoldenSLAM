#include "platform/ros_bridge.hpp"
#include "core/slam_core.hpp"

using namespace std::chrono_literals;

RosBridge::RosBridge(SlamCore* core)
    : rclcpp::Node("golden_slam"), core_(core)
{
    RCLCPP_INFO(this->get_logger(), "RosBridge started.");
    std::cout << "RosBridge started"<< std::endl;

}

void RosBridge::onFrontendTimer()
{
    RCLCPP_INFO(this->get_logger(), "Frontend timer tick.");

}