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
    auto lidar_qos = rclcpp::QoS(rclcpp::KeepLast(100)).best_effort();
    auto imu_qos   = rclcpp::QoS(rclcpp::KeepLast(5000)).best_effort();

    lidar_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        lidar_topic_,
        lidar_qos,
        std::bind(&RosBridge::onLidarCB,this, std::placeholders::_1)
    );
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_,
        imu_qos,
        std::bind(&RosBridge::onImuCB,this, std::placeholders::_1)
    );



}
void RosBridge::setupPublishers()
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/preprocess/lidar", qos);

    // 같은 frame pair 비교용
    debug_preprocess_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/debug/preprocess_lidar", qos);

    undistorted_pub_  = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/debug/undistorted_lidar", qos);

    world_frame_pub_ =this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/debug/world_frame_cloud", qos);
    
    accumulated_map_pub_ =this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/debug/accumulated_map", qos);
    
}

void RosBridge::setupTimer()
{

    auto frontend_period =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(10.0 / 1000.0));
    auto map_publish_period =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(2000.0 / 1000.0));

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
    core_->spinFrontendOnce();

    // auto preprocess_cloud = core_->getDebugPreprocessCloud();
    auto undistorted_cloud = core_->getUndistortedCloud();
    // auto world_frame_cloud = core_->getWorldFrameCloud();
    // auto accumulated_map_cloud = core_->getAccumulatedMapCloud();



    const double stamp_sec = core_->getLastProcessedFrameTime();
    const auto sec = static_cast<int32_t>(stamp_sec);
    const auto nanosec =
        static_cast<uint32_t>((stamp_sec - static_cast<double>(sec)) * 1e9);
//  if (preprocess_cloud && !preprocess_cloud->points.empty()) {
//         sensor_msgs::msg::PointCloud2 msg;
//         pcl::toROSMsg(*preprocess_cloud, msg);
//         msg.header.stamp.sec = sec;
//         msg.header.stamp.nanosec = nanosec;
//         msg.header.frame_id = "map";
//         debug_preprocess_pub_->publish(msg);
//     }

    if (undistorted_cloud && !undistorted_cloud->points.empty()) {
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*undistorted_cloud, msg);
        msg.header.stamp.sec = sec;
        msg.header.stamp.nanosec = nanosec;
        msg.header.frame_id = "map";
        undistorted_pub_->publish(msg);
    }

    // if (world_frame_cloud && !world_frame_cloud->points.empty()) {
    //     sensor_msgs::msg::PointCloud2 msg;
    //     pcl::toROSMsg(*world_frame_cloud, msg);
    //     msg.header.stamp.sec = sec;
    //     msg.header.stamp.nanosec = nanosec;
    //     msg.header.frame_id = "map";
    //     world_frame_pub_->publish(msg);
    // }

    // // if (accumulated_map_cloud && !accumulated_map_cloud->points.empty()) {
    //     sensor_msgs::msg::PointCloud2 msg;
    //     pcl::toROSMsg(*accumulated_map_cloud, msg);
    //     msg.header.stamp.sec = sec;
    //     msg.header.stamp.nanosec = nanosec;
    //     msg.header.frame_id = "map";
    //     accumulated_map_pub_->publish(msg);
    // }
    
}
void RosBridge::onMapPublishTimer()
{
    // std::cout << "444" << std::endl;
    //나중에 core에서 map snapshot 이미 만들어진 것을 요청하고 여기서 publish할것임. 
    auto accumulated_map_cloud = core_->getAccumulatedMapCloud();

    if (!accumulated_map_cloud || accumulated_map_cloud->points.empty()) {
        return;
    }

    const double stamp_sec = core_->getLastProcessedFrameTime();

    const auto sec = static_cast<int32_t>(stamp_sec);
    const auto nanosec =
        static_cast<uint32_t>((stamp_sec - static_cast<double>(sec)) * 1e9);

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*accumulated_map_cloud, msg);
    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
    msg.header.frame_id = "map";

    accumulated_map_pub_->publish(msg);
}

void RosBridge::onImuCB(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // std::cout << "222" << std::endl;
    core_->pushImuMsg(msg);
}
void RosBridge::onLidarCB(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
{

     auto cloud = preprocess_.lidarConvert(msg);


     sensor_msgs::msg::PointCloud2 cloud_msg;
     pcl::toROSMsg(*cloud, cloud_msg);

     cloud_msg.header.stamp = msg->header.stamp;
    //  cloud_msg.header.frame_id = "livox_frame"; //일단 센서 프레임
     cloud_msg.header.frame_id = "map"; //일단 센서 프레임
     lidar_pub_->publish(cloud_msg);

    LidarFrame lidar_frame;
    lidar_frame.frame_beg_time = 
                    static_cast<double>(msg->header.stamp.sec) +
                    static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    lidar_frame.cloud = cloud;
    core_->pushLidarFrame(lidar_frame); //나중에 core로 넘기기
}


void RosBridge::mapSaveCB(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)
{
    // std::cout << "mapSaveCB" << std::endl;
}
