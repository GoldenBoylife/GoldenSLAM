#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <chrono>




class SlamCore;

class RosBridge : public rclcpp::Node
{

public:
    explicit RosBridge(SlamCore* core);
    //명백한 이런 형태로만 선언
private: 
    void loadParameters();
    void setupSubscribers();
    void setupTimer();
    void setupServices();

    void onFrontendTimer(); // 100hz
    void onMapPublishTimer(); // rviz에서 보여지기 위함 1hz

    void onImuCB(const sensor_msgs::msg::Imu::SharedPtr msg);
    void onLidarCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg);


private:
    SlamCore* core_;
    //나는 소유자는 아니고, 누가 만든거 가져다 쓰겠다는 뜻
    // 진짜 주인은 GoldenSlamApp임

    std::string imu_topic_;
    std::string lidar_topic_;
    std::string lidar_msg_type_; //pointcloud2 or livox_custom
    std::string sensor_model_; //mid360, avia
    std::string map_frame_;
    std::string base_frame_;

    double frontend_rate_;
    double map_publish_rate_;



    rclcpp::TimerBase::SharedPtr frontend_timer_;
    rclcpp::TimerBase::SharedPtr map_publish_timer_;

    rclcpp:Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;

};