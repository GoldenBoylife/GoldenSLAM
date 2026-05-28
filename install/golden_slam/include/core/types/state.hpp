#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

struct State
{
    double timestamp  =0.0;

    /*world 기준 position / velocity*/
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();

    /*body -> world rotation*/
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();

    /*bias는 일단 0으로 시작*/
    Eigen::Vector3d gyr_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_bias = Eigen::Vector3d::Zero();

};