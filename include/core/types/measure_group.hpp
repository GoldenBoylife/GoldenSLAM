#pragma once

#include "core/types/lidar_frame.hpp"
#include <deque>
struct MeasureGroup 
{
    LidarFrame lidar_frame;
    std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imus; 
};
