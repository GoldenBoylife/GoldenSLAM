#include "app/golden_slam_app.hpp"
#include "core/slam_core.hpp"
#include "platform/ros_bridge.hpp"

GoldenSlamApp::GoldenSlamApp(int argc, char** argv)
    :argc_(argc), argv_(argv)
{
    
}

GoldenSlamApp::~GoldenSlamAPP()
{
    if(rclcpp::ok())
        rclcpp::shutdown();
}

int GoldenSlamApp::run() 
{
    slamCore_ = std::make_shared<SlamCore>();
    rclcpp::int(argc_,argv_);
    rosBridge_ = std::make_shared<RosBridge>(slamCore_.get());
    //unique 포인터라서, raw point를 주기위해서 .get씀
    std::cout << "GoldenSlamApp started" << std::endl;
    rclcpp::spin(rosBridge_);
    rclcpp::shutdown();

    return 0;
}