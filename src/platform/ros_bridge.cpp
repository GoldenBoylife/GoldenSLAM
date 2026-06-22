
#include "platform/ros_bridge.hpp"
#include "core/slam_core.hpp"
#include <builtin_interfaces/msg/time.hpp>

#include "core/types/lidar_frame.hpp"


static double stamp_to_sec(const builtin_interfaces::msg::Time& t)
{
    return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

builtin_interfaces::msg::Time secToRosTime(double stamp_sec)  
{
    builtin_interfaces::msg::Time t;

    const double sec_floor = std::floor(stamp_sec);

    t.sec = static_cast<int32_t>(sec_floor);
    t.nanosec = static_cast<uint8_t>((stamp_sec - sec_floor) * 1e9);

    return t;
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


    if(core_ !=nullptr) 
    {
        core_->setParams(sp);
    }
}




void RosBridge::setupSubscribers()
{

    // auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(2000)).best_effort();

    // lidar_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
    //     lidar_topic_,
    //     20,
    //     std::bind(&RosBridge::onLidarCB,this,std::placeholders::_1)
    // );
    // imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    //     imu_topic_,
    //     10,
    //     std::bind(&RosBridge::onImuCB,this,std::placeholders::_1)
    // );

    auto lidar_qos = rclcpp::QoS(rclcpp::KeepLast(200)).best_effort();
    auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(2000)).best_effort();

    lidar_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        lidar_topic_,
        lidar_qos,
        std::bind(&RosBridge::onLidarCB, this, std::placeholders::_1)
    );

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_,
        imu_qos,
        std::bind(&RosBridge::onImuCB, this, std::placeholders::_1)
    );
}

void RosBridge::setupPublishers()
{
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/Odometry",
        20
    );


    pub_raw_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/cloud_raw",
        20
    );

    pub_undistort_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/cloud_undistort",
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

    pub_debug_map_predicted_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
         "/golden_slam/debug_map_predicted",
        20
    );
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

     static double last_raw_imu_time = 0.0;
    static int imu_count = 0;
    static int gap_count = 0;
    static double max_raw_dt = 0.0;
    static double sum_raw_dt = 0.0;


    if(last_raw_imu_time > 0.0)
    {
        const double raw_dt = imu.timestamp - last_raw_imu_time;

        sum_raw_dt += raw_dt;

        if (raw_dt > max_raw_dt)
        {
            max_raw_dt = raw_dt;
        }

        if (raw_dt > 0.02)
        {
            ++gap_count;
        }
    }

    last_raw_imu_time = imu.timestamp;
    ++imu_count;

    if (imu_count % 200 == 0)
    {
        const double avg_raw_dt = sum_raw_dt / static_cast<double>(imu_count - 1);

        std::cout << "[RosBridge::onImuCB][raw imu stats] "
                  << " count=" << imu_count
                  << " avg_dt=" << avg_raw_dt
                  << " max_dt=" << max_raw_dt
                  << " gap_count=" << gap_count
                  << std::endl;

        gap_count = 0;
        max_raw_dt = 0.0;
        sum_raw_dt = 0.0;
        imu_count = 0;
    }

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

    SlamSnapShot snapshot;
    if(!core_->popSnapshot(snapshot))
    {
        return;
    }

    publishCloudBody(
        snapshot.cloud_raw,
        "map",
        snapshot.lidar_end_time,
        pub_raw_
    );
    publishCloudBody(
        snapshot.cloud_undistort,
        "map",  //ori : camera_init
        snapshot.lidar_end_time,
        pub_undistort_
    );

    if(snapshot.cloud_map_predicted && !snapshot.cloud_map_predicted->empty())
    {
        publishCloudBody(
            snapshot.cloud_map_predicted,
            "map",
            snapshot.lidar_end_time,
            pub_debug_map_predicted_
        );
    }


    static int pub_count = 0;
    ++pub_count;
    if (pub_count % 10 == 0)
    {
        const std::size_t raw_size =
            snapshot.cloud_raw ? snapshot.cloud_raw->size() : 0;

        const std::size_t undistort_size =
            snapshot.cloud_undistort ? snapshot.cloud_undistort->size() : 0;

        const std::size_t map_size =
            snapshot.cloud_map_predicted ? snapshot.cloud_map_predicted->size() : 0;

        std::cout << "[RosBridge::onFrontendTimer][publish snapshot] "
                << " count=" << pub_count
                << " raw=" << raw_size
                << " undistort=" << undistort_size
                << " debug_map=" << map_size
                << " stamp=" << snapshot.lidar_end_time
                << std::endl;
    }
}

void RosBridge::publishCloudBody(
    const CloudTPtr cloud,
    const std::string& frame_id,
    double stamp_sec,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub) 
{
    if(!cloud || cloud->empty())        return;

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);

    cloud_msg.header.frame_id = frame_id;
    cloud_msg.header.stamp = secToRosTime(stamp_sec);

    pub->publish(cloud_msg);
}



void RosBridge::onMapPubTimer()
{

}