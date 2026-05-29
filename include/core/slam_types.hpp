#pragma once
// 경량 타입 — RosBridge 가 이 헤더만 보면 됨 (알고리즘 헤더 불필요)

#include <string>
#include <vector>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/msg/quaternion.hpp>

// ── ROS 없는 IMU 데이터 (RosBridge 가 변환 담당) ────────────────────────────
struct ImuData
{
    double          timestamp           = 0.0;
    Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity    = Eigen::Vector3d::Zero();
};

// ── 외부 공개용 Pose (알고리즘 타입 없음) ────────────────────────────────────
struct PoseState
{
    Eigen::Vector3d    pos            = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rot            = Eigen::Quaterniond::Identity();
    Eigen::Vector3d    vel            = Eigen::Vector3d::Zero();

    /*LiDAR와 IMU 사이 extrinsic*/
    Eigen::Matrix3d    offset_R_L_I   = Eigen::Matrix3d::Identity();
    //offset_T_L_I : LiDAR 원점이 IMU 기준으로 어느 회전?
    Eigen::Vector3d    offset_T_L_I   = Eigen::Vector3d::Zero();
    //offset_T_L_I : LiDAR 원점이 IMU 기준으로 어디 있는지?
};

// ── ROS 파라미터 → SlamCore 로 전달할 설정 ──────────────────────────────────
struct SlamParams
{
    double filter_size_surf_min  = 0.5;
    double filter_size_map_min   = 0.5;
    
    double box_len              = 200.0;
    float  det_range             = 300.0f;
    double fov_deg               = 180.0;
    double gyr_cov               = 0.1;
    double acc_cov               = 0.1;
    double b_gyr_cov             = 0.0001;
    double b_acc_cov             = 0.0001;
    std::vector<double> extrinT  = {0.0, 0.0, 0.0};
    std::vector<double> extrinR  = {1,0,0, 0,1,0, 0,0,1};
    bool   extrinsic_est_en      = true;
    int    num_max_iterations    = 4;
    bool   time_sync_en          = false;
    double time_diff_lidar_to_imu = 0.0;
    bool   runtime_pos_log       = false;
    bool   pcd_save_en           = false;
    int    pcd_save_interval     = -1;
    std::string map_file_path    = "";
    std::string root_dir         = "";
};

// ── SlamCore → RosBridge 로 전달하는 스냅샷 ─────────────────────────────────
struct SlamSnapshot
{
    bool   valid            = false;
    double lidar_end_time   = 0.0;

    PoseState                  pose;
    geometry_msgs::msg::Quaternion geoQuat;
    Eigen::Matrix<double, 23, 23>  cov_P = Eigen::Matrix<double, 23, 23>::Zero();

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZINormal>> feats_undistort;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZINormal>> feats_down_body;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZINormal>> laserCloudOri;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZINormal>> pcl_wait_pub;

    int  effct_feat_num = 0;
    bool dense_pub_en   = false;
    bool scan_pub_en    = false;
};
