#pragma once

#include <iostream>

#include <deque>
#include <mutex>

#include "core/slam_core.hpp"
#include "core/types/pcl_types.hpp"
#include <sensor_msgs/msg/imu.hpp>

struct LidarFrame
{
    double frame_end_time =0.0; 
    double frame_beg_time =0.0; 
    CloudTPtr cloud = nullptr; 

};

struct MeasureGroup 
{
    LidarFrame lidar_frame;
    std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imus; 
};


class SlamCore
{
public:
    SlamCore();
    ~SlamCore();
    void spinFrontendOnce();
    /*push*/

    void pushLidarFrame(const LidarFrame& frame);
    //const라서 가리키는 녀석은 바꿀수 없지만, 
    //가리키는 녀석의 .poinsts같은 건 편집 가능
    //예 cloud.reset() 같은 건 불가, cloud->points.clear()가능
    void pushImu(const sensor_msgs::msg::Imu::SharedPtr msg);
    /*      push*/

private: 
    /*syncMeasure*/
    bool syncMeasure(MeasureGroup& meas);
    bool isValidBuffer();
    void debugImuDt(const double imu_time);
    // bool isValidLidarHz();
    /*          syncMeasure*/
    

private: 
    bool is_first_lidar_;
    bool lidar_frame_pushed_;
    
    /*syncMeasure*/
    
    std::mutex buffer_mutex_;
    std::deque<LidarFrame> lidar_frame_buffer_;
    std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer_;
    bool has_last_imu_time_ =false;
    double last_timestamp_imu_=0.0;
    double last_imu_time_ =0.0;
    std::size_t imu_count_  = 0;
    LidarFrame current_lidar_frame_;
        /*          syncMeasure*/


};