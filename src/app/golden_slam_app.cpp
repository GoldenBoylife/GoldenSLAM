#include "app/golden_slam_app.hpp"
#include "core/slam_core.hpp"
#include "platform/ros_bridge.hpp"


GoldenSlamApp::GoldenSlamApp(int argc, char** argv)
    : argc_(argc), argv_(argv)
{


}
GoldenSlamApp::~GoldenSlamApp()
{
    if(rclcpp::ok())
        rclcpp::shutdown();
}

int GoldenSlamApp::run()
{
    slamCore_ = std::make_unique<SlamCore>();
    rclcpp::init(argc_,argv_);
    rosBridge_ = std::make_shared<RosBridge>(slamCore_.get());
    //unique 포인터라서, raw pointer를 주기위해서 .get()씀
    
    rclcpp::spin(rosBridge_);
    rclcpp::shutdown();
    return 0;
    
}