#pragma once

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

struct ImuPropagatedPose
{
    double timestamp = 0.0;

    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
};

using ImuPropagatedPoseHistory = std::vector<ImuPropagatedPose>;