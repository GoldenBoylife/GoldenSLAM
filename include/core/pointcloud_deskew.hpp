#pragma once

#include "core/types/imu_propagated_pose.hpp"
#include "core/types/lidar_frame.hpp"
#include "core/types/pcl_types.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>

class PointCloudDeskew
{
public:
    CloudTPtr deskew( const LidarFrame& lidar_frame,const ImuPropagatedPoseHistory& imu_pose_history) const;


private:
    bool findInterpolatedPose(double query_time, const ImuPropagatedPoseHistory& history, ImuPropagatedPose& out_pose) const;

    
};