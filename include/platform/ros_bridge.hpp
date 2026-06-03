#pragma once 
#include <memory>
#include <rclcpp/rclcpp.hpp>


#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>



#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "core/types/slam_types.hpp"
#include "core/preprocess.hpp"


class SlamCore;
class Preprocess;
class RosBridge : public rclcpp::Node 
{
public:

    RosBridge(SlamCore* core);
    /*ROS runtime life위해서 GoldenSlamApp에서만 호출*/
    static void rosInit(int argc, char** argv);
    static void rosShutdown();
    void spin();

private:
    /*setup*/
    void loadParameters();


    void setupSubscribers();
    void setupPublishers();
    void setupTimer();
    void setupService();
    void setupPath();

    /*callbacks*/
    void onImuCB(sensor_msgs::msg::Imu::SharedPtr msg_in);
    void onLidarCB(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
    void mapSaveCB(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);

    /*timer*/
    void onFrontendTimer();     //100Hz
    void onMapPubTimer(); // rviz에서 

    /*publish*/
    void pubFrameBody(const SlamSnapShot& snap);    //현재 lidar frame의 body기준
    void pubFrameWorld(const SlamSnapShot& snap);   //현재 lidar frame의 world기준
    void pubOdometry(const SlamSnapShot& snap); //현재 추정된 로봇 pose를 publish
    void pubPath(const SlamSnapShot& snap); //odometry를 쌓아서 궤적으로 보여줌
    void pubMap(const SlamSnapShot& snap);


private:
    SlamCore* core_;
    //소유자 아니고, 누가 만든 것을 가져다 쓰겠다는 뜻, 진짜 소유자는 GoldenSlamApp

    Preprocess preprocess_;

    std::string lidar_topic_;
    std::string imu_topic_;

    /*topic sub*/
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    /*topic pub*/
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_ ;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_frame_body_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr               pub_path_;


    /*timer*/
    rclcpp::TimerBase::SharedPtr frontend_timer_;
    rclcpp::TimerBase::SharedPtr backend_timer_; //TODO
    rclcpp::TimerBase::SharedPtr map_pub_timer_;

    /*services*/
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;

    /*path accumulation*/
    nav_msgs::msg::Path path_;
    
    /*tf*/
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
};