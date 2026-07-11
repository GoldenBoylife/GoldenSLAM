#pragma once

#include <string>
#include <vector>
#include <memory>
#include <deque>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include "core/types/pcl_types.hpp"
#include "core/types/lidar_frame.hpp"


// struct SlamParams
// {

// };

typedef Eigen::Vector3d V3D;
typedef Eigen::Matrix3d M3D;
typedef Eigen::Vector3f V3F;
typedef Eigen::Matrix3f M3F;
using Q12 = Eigen::Matrix<double,12,12>;



struct ImuData
{
    double  timestamp = 0.0;
    Eigen::Vector3d linear_acc  = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_vel     = Eigen::Vector3d::Zero();
};


struct PoseState
{
    Eigen::Vector3d         pos         = Eigen::Vector3d::Zero();
    Eigen::Quaterniond      rot      = Eigen::Quaterniond::Identity();
    Eigen::Vector3d         vel         = Eigen::Vector3d::Zero();

    /*bias는 일단 0으로 시작*/
    Eigen::Vector3d gyr_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_bias = Eigen::Vector3d::Zero();

    Eigen::Vector3d gravity =   Eigen::Vector3d(0.0, 0.0, -9.80665);
    //z-up 좌표계라면  이렇게 씀

    /*LiDAR와 imu 사이 extrinsic*/
    //처음엔 상수지만, 필요하면 EKF state 안에서 같이 보정 됨
    Eigen::Matrix3d         offset_R_L_I = Eigen::Matrix3d::Identity();
    //offset_R_L_I : LiDAR 원점이 IMU 기준으로 어느 회전
    Eigen::Vector3d         offset_T_L_I = Eigen::Vector3d::Zero();
    //offset_T_L_I : LiDAR 원점이 IMU 기준으로 어디 잇는지?

};

/*for debug*/
struct SlamSnapShot
{
    bool valid = false;
    double lidar_beg_time = 0.0; //
    double lidar_end_time = 0.0; //이번 프레임 시간
    PoseState                  state;
    CloudTPtr   cloud_raw;      //원본
    CloudTPtr   cloud_undistorted;    //LiDAR body기준 cloud

    CloudTPtr cloud_map_predicted; 
};


// ── ROS 파라미터 → SlamCore 로 전달할 설정 ──────────────────────────────────
struct SlamParams
{
    // double filter_size_surf_min  = 0.5;
    // double filter_size_map_min   = 0.5;
    
    // double box_len              = 200.0;
    // float  det_range             = 300.0f;
    // double fov_deg               = 180.0;
    double gyr_cov               = 0.1;
    double acc_cov               = 0.1;
    double b_gyr_cov             = 0.0001;
    double b_acc_cov             = 0.0001;
    std::vector<double> extrinT  = {0.0, 0.0, 0.0};
    std::vector<double> extrinR  = {1,0,0, 0,1,0, 0,0,1};
    // bool   extrinsic_est_en      = true;
    int    num_max_iterations    = 4;
    // bool   time_sync_en          = false;
    // double time_diff_lidar_to_imu = 0.0;
    // bool   runtime_pos_log       = false;
    // bool   pcd_save_en           = false;
    // int    pcd_save_interval     = -1;
    // std::string map_file_path    = "";
    // std::string root_dir         = "";
};



struct MeasureGroup
{
    LidarFrame lidar_frame;
    std::deque<ImuData> imus;
};


struct PlaneResidual
{
    PointT point_world;
    Eigen::Vector3d normal;
    Eigen::Vector3d center;

    double residual = 0.0;
    double abs_residual = 0.0;
    double dist5 = 0.0;
};
struct IkdTreeProcessResult
{
    bool map_initialized_this_frame = false;
    bool should_add_to_map = false;

    std::vector<PlaneResidual> update_residuals;

    int search_found = 0;
    int search_fail = 0;

    int residual_candidate = 0;
    int distance_reject = 0;

    int plane_ok = 0;
    int plane_fail = 0;

    int residual_update_candidate = 0;
    int residual_reject = 0;

    double avg_dist5 = 0.0;
    double max_dist5 = 0.0;

    double avg_abs_residual = 0.0;
    double max_abs_residual = 0.0;

    double avg_update_abs_residual = 0.0;
    double max_update_abs_residual = 0.0;
};


struct PoseCorrectionResult
{
    bool valid = false;

    Eigen::Matrix<double, 6, 1> dx =
        Eigen::Matrix<double, 6, 1>::Zero();

    int update_count = 0;

    double rot_norm = 0.0;
    double trans_norm = 0.0;
};