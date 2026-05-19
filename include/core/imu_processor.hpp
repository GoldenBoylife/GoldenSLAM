#pragma once

#include <sensor_msgs/msg/imu.hpp>

#include "core/types/measure_group.hpp"
#include "core/types/state.hpp"

/*Imu값을 입력 받고 적분하여서 pose를 상태예측*/
//rotation
//velocity
//position
// gyr -> rotation update , acc -> velocity / position update

static constexpr int INIT_FRAME_NUM =20;


class ImuProcessor
{
public: 
    ImuProcessor();
    bool initImuProcessor(const MeasureGroup& meas, State& state);
    bool isInitialized() const;



    /*imu_propagate*/
    void propagate(const MeasureGroup& meas, State& state);
    //rotation예측
    //velocity예측
    //position예측

    double getImuTime(const sensor_msgs::msg::Imu::ConstSharedPtr& imu) const;
    Eigen::Vector3d getGyr(const sensor_msgs::msg::Imu::ConstSharedPtr& imu) const;
    Eigen::Vector3d getAcc(const sensor_msgs::msg::Imu::ConstSharedPtr& imu) const;
    Eigen::Quaterniond deltaQ(const Eigen::Vector3d& omega, double dt) const;
    void scaleGravity();
    /*      imu_propagate*/

private:
    bool initialized_;
    /*imu_propagate*/
    Eigen::Vector3d gravity_ ;
    int init_frame_count_ ;
    int init_sample_count_ ;

    Eigen::Vector3d mean_acc_;
    Eigen::Vector3d mean_gyr_;


    double acc_scale_ ; //gravity scale

    /*      imu_propagate*/

};