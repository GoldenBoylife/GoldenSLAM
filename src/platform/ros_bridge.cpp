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
    setupPublishers();
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

void RosBridge::setupPublishers() 
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/preprocess/lidar", qos);
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
    core_->spinFrontendOnce();
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

    
    auto preprocess_result = preprocess_.lidarConvert(msg);
    auto cloud = preprocess_result .cloud;               
    if(!cloud || cloud->points.empty())
    {
        std::cout << "[onLidarCB] empty cloud" << std::endl;
        return;
    }

    LidarFrame lidarframe;
    lidarframe.cloud = cloud;
    lidarframe.frame_beg_time =
        rclcpp::Time(msg->header.stamp).seconds();
    lidarframe.frame_end_time = 
        lidarframe.frame_beg_time + preprocess_result .max_relative_time;




    sensor_msgs::msg::PointCloud2 cloud_msg;

    /*debug*/
    const auto raw_size = msg->point_num;
    const auto preprocced_size = cloud->points.size();

    std::cout << 
    "raw : " << raw_size << " processed : " <<  preprocced_size << std::endl;

    std::cout
        << "[onLidarCB] raw=" << msg->point_num
        << ", processed=" << cloud->points.size()
        << ", beg=" << std::fixed << std::setprecision(9)
        << lidarframe.frame_beg_time
        << ", end=" << lidarframe.frame_end_time
        << ", duration=" << preprocess_result .max_relative_time
        << std::endl;

    /*     debug*/

    // cloud_msg.header.stamp = msg->header.stamp;
    // cloud_msg.header.frame_id = "map";
    // lidar_pub_->publish(cloud_msg);

    core_->pushLidarFrame(lidarframe);
}


void RosBridge::mapSaveCB(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)
{
    std::cout << "mapSaveCB" << std::endl;

}
