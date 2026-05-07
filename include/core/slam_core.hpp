#pragma once

#include <iostream>

#include <deque>

#include "core/slam_core.hpp"
#include "core/types/pcl_types.hpp";
#include <sensor_msgs/msg/imu.hpp>


struct MeasureGroup 
{
    double frame_end_time =0.0; 
    double frame_beg_time =0.0; 
    CloudTPtr cloud = std::make_shared<CloudT>(); 
    std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imus; 
};


class SlamCore
{
public:
    SlamCore();
    ~SlamCore();
    void spinFrontendOnce();

private: 
    /*syncMeasure*/
    bool syncMeasure(MeasureGroup& meas);
    bool isValidBuffer();
    /*          syncMeasure*/
    

private: 
    std::deque<CloudTPtr> lidar_buffer_;
    std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer_;

};