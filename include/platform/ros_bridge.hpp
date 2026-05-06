#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include <std_srvs/srv/trigger.hpp>

#include "core/preprocess.hpp" //전방 선언 아님, 소유권은 RosBridge가 가짐





class SlamCore;
class Preprocess;
class RosBridge : public rclcpp::Node
{

public:
    explicit RosBridge(SlamCore* core);
    //명백한 이런 형태로만 선언
private: 
    void loadParameters();
    void setupSubscribers();
    void setupPublishers();
    void setupTimer();
    void setupServices();

    void onFrontendTimer(); // 100hz
    void onMapPublishTimer(); // rviz에서 보여지기 위함 1hz

    void onImuCB(const sensor_msgs::msg::Imu::SharedPtr msg);
    // void onLidarCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void onLidarCB(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
    void mapSaveCB(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);
    

private:
    SlamCore* core_;
    //나는 소유자는 아니고, 누가 만든거 가져다 쓰겠다는 뜻
    // 진짜 주인은 GoldenSlamApp임
    Preprocess preprocess_;

    std::string imu_topic_;
    std::string lidar_topic_;
    std::string lidar_msg_type_; //pointcloud2 or livox_custom
    std::string sensor_model_; //mid360, avia
    std::string map_frame_;
    std::string base_frame_;

    double frontend_rate_;
    double map_publish_rate_;



    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;


    /*topic sub*/
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    /*topic pub*/
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_preprocess_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr undistorted_pub_;

        /*poseEstimate */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr world_frame_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr accumulated_map_pub_;

    /*timer*/
    rclcpp::TimerBase::SharedPtr frontend_timer_;
    rclcpp::TimerBase::SharedPtr map_publish_timer_;





};