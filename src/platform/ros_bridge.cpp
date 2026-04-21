#include "platform/ros_bridge.hpp"
#include "core/slam_core.hpp"

using namespace std::chrono_literals;

RosBridge::RosBridge(SlamCore* core)
    : rclcpp::Node("golden_slam"), core_(core)
{
    RCLCPP_INFO(this->get_logger(), "RosBridge started.");
    std::cout << "RosBridge started"<< std::endl;

    loadParameters();
    setupSubscribers();
    setupTimer();
    setupServices();
}




void RosBridge::loadParameters()
{
    imu_topic_ =  "/livox/imu";
    lidar_topic_ =  "/livox/lidar";
}

void RosBridge::setupSubscribers()
{


    lidar_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        lidar_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&RosBridge::onLidarCB,this, std::placeholders::_1)
    );
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&RosBridge::onImuCB,this, std::placeholders::_1)
    );



}

void RosBridge::setupTimer()
{

    auto frontend_period =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(10.0 / 1000.0));
    auto map_publish_period =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1000.0 / 1000.0));

    frontend_timer_ = rclcpp::create_timer(this, this->get_clock(),frontend_period, 
                        std::bind(&RosBridge::onFrontendTimer,this));
    map_publish_timer_ = rclcpp::create_timer(this, this->get_clock(),map_publish_period,
                        std::bind(&RosBridge::onMapPublishTimer,this));
}

void RosBridge::setupServices()
{
    map_save_srv_ = this->create_service<std_srvs::srv::Trigger>("map_save", std::bind(&RosBridge::mapSaveCB,
                    this, std::placeholders::_1, std::placeholders::_2));
    // map_save_srv_ = this->create_service<std_srvs::srv::Trigger>("map_save", std::bind(&LaserMappingNode::map_save_callback, 
    //     this, std::placeholders::_1, std::placeholders::_2));

}



void RosBridge::onFrontendTimer()
{
    RCLCPP_INFO(this->get_logger(), "Frontend timer tick.");
    // std::cout << "333" << std::endl;
}

void RosBridge::onMapPublishTimer()
{
    // std::cout << "444" << std::endl;
}

void RosBridge::onImuCB(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // std::cout << "222" << std::endl;
}
void RosBridge::onLidarCB(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
{
    // std::cout << "111" << std::endl;
}


void RosBridge::mapSaveCB(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)
{
    std::cout << "mapSaveCB" << std::endl;
}
