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
    LidarFrame lidarframe;
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
    /*      push*/

private: 
    /*syncMeasure*/
    bool syncMeasure(MeasureGroup& meas);
    bool isValidBuffer();

    /*          syncMeasure*/
    

private: 
    bool is_first_lidar_;
    bool lidar_pushed_;
    std::deque<LidarFrame> lidar_frame_buffer_;
    std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer_;
    
    /*syncMeasure*/
    std::mutex buffer_mutex_;
    /*          syncMeasure*/


};