#pragma once

#include <iostream>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include <deque>
#include <mutex>


#include <core/types/pcl_types.hpp>


struct MeasureGroup
{
    // livox_ros_driver2::msg::CustomMsg::SharedPtr lidar_msg;
    LidarFrame lidar_frame;
    double frame_end_time =0.0;
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imus;
};


class SlamCore
{
public:
    SlamCore();
    ~SlamCore();

    void pushImuMsg(const sensor_msgs::msg::Imu::SharedPtr msg);
    void pushLidarFrame(const LidarFrame& frame );
    void spinFrontendOnce();
    bool saveMap();

private:
        double toSec(const builtin_interfaces::msg::Time& stamp) const ;
    bool syncMeasureGroup(MeasureGroup& meas);

    bool hasSensorBuffer() const ;
    bool prepareLidarFrame(MeasureGroup& meas);
    bool hasEnoughImuForCurrentLidarFrame() const ;
    void collectImuForCurrentLidarFrame(MeasureGroup& meas);
    void consumeCurrentLidarFrame();


private: 
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_buffer_;
    // std::deque<livox_ros_driver2::msg::CustomMsg::SharedPtr> lidar_buffer_;
    // std::deque<double> lidar_time_buffer_;
    std::deque<LidarFrame> lidar_frame_buffer_;

    std::mutex buffer_mutex_;
    

    double last_timestamp_imu_ = 0.0;
    double last_timestamp_lidar_ = 0.0;

    double lidar_end_time_ = 0.0;
    double lidar_mean_scantime_ = 0.1;
    int scan_num =0;
    bool lidar_pushed_ = false;


};