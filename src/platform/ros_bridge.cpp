
#include "platform/ros_bridge.hpp"
#include "core/slam_core.hpp"
#include <builtin_interfaces/msg/time.hpp>

#include "core/types/lidar_frame.hpp"


static double stamp_to_sec(const builtin_interfaces::msg::Time& t)
{
    return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

RosBridge::RosBridge(SlamCore* core) 
    :   rclcpp::Node("laser_mapping"),
        core_(core)
{
    loadParameters();
    setupSubscribers();
    setupPublishers();
    setupTimer();
    setupService();
    setupPath();


    RCLCPP_INFO(this->get_logger(), "RosBridge init finished.");
}


void RosBridge::rosInit(int argc, char** argv)
{
    rclcpp::init(argc,argv);
}

void RosBridge::rosShutdown()
{
    if(rclcpp::ok()) rclcpp::shutdown();
}


void RosBridge::spin()
{
    rclcpp::spin(shared_from_this());
    //
    rclcpp::shutdown();
}

/*setup */
void RosBridge::loadParameters()
{
    SlamParams sp;
    lidar_topic_ = this->declare_parameter<std::string>("common.lid_topic", "/livox/lidar");
    imu_topic_ = this->declare_parameter<std::string>("common.imu_topic", "/livox/imu");

    // std::cout << "lid_topic : " << lidar_topic_ << ", imu_topic : "<< imu_topic_ << std::endl;

    RCLCPP_INFO(this->get_logger(), "lidar_topic: %s", lidar_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "imu_topic: %s", imu_topic_.c_str());
    
    sp.extrinT = this->declare_parameter<std::vector<double>>("mapping.extrinsic_T");
    sp.extrinR = this->declare_parameter<std::vector<double>>("mapping.extrinsic_R");
}




void RosBridge::setupSubscribers()
{

    auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(2000)).best_effort();

    lidar_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        lidar_topic_,
        20,
        std::bind(&RosBridge::onLidarCB,this,std::placeholders::_1)
    );
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_,
        10,
        std::bind(&RosBridge::onImuCB,this,std::placeholders::_1)
    );
}

void RosBridge::setupPublishers()
{
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/Odometry",
        20
    );

    pub_frame_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/cloud_deskewed_body",
        20
    );
    //deskew된 현재 LiDAr frame 확인

    pub_path_ = this->create_publisher<nav_msgs::msg::Path>(
        "/path",
        20
    );
    //누적 궤적

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    //rviz frame 관계 확인 
}

void RosBridge::setupTimer()
{
    auto frontend_hz = 100.0;
    auto map_pub_hz = 1.0;
    auto frontend_period = std::chrono::milliseconds(static_cast<int64_t>(1000.0/frontend_hz));
    auto map_pub_period = std::chrono::milliseconds(static_cast<int64_t>(1000.0/map_pub_hz));

    frontend_timer_ = rclcpp::create_timer(this, this->get_clock(), frontend_period,
        std::bind(&RosBridge::onFrontendTimer,this));
    
    map_pub_timer_ = rclcpp::create_timer(this, this->get_clock(), map_pub_period,
        std::bind(&RosBridge::onMapPubTimer, this));
}

void RosBridge::setupService()
{
  map_save_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "map_save", 
    std::bind(&RosBridge::mapSaveCB,
                    this, std::placeholders::_1, std::placeholders::_2));
}

void RosBridge::setupPath()
{
    path_.header.stamp    = this->get_clock()->now();
    path_.header.frame_id = "camera_init";
}


void RosBridge::onImuCB(sensor_msgs::msg::Imu::SharedPtr msg_in)
{
    ImuData imu;
    imu.timestamp = stamp_to_sec(msg_in->header.stamp);
    imu.linear_acc << msg_in->linear_acceleration.x,
                      msg_in->linear_acceleration.y,
                      msg_in->linear_acceleration.z;
                      
    imu.angular_vel << msg_in->angular_velocity.x,
                      msg_in->angular_velocity.y,
                      msg_in->angular_velocity.z;
    core_->pushImu(imu);


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
    core_->pushLidarFrame(lidarframe);

}

void RosBridge::mapSaveCB(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)
{

}


void RosBridge::onFrontendTimer()
{
    core_->spinFrontendOnce();
}

void RosBridge::onMapPubTimer()
{

}