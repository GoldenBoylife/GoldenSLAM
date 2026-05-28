#pragma once

#include <mutex>
#include <string>
#include <vector>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ─── Minimal pose state for publishing (no algorithm headers needed) ──────────
struct PoseState
{
    Eigen::Vector3d    pos            = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rot            = Eigen::Quaterniond::Identity();
    Eigen::Vector3d    vel            = Eigen::Vector3d::Zero();
    Eigen::Matrix3d    offset_R_L_I   = Eigen::Matrix3d::Identity();
    Eigen::Vector3d    offset_T_L_I   = Eigen::Vector3d::Zero();
};

// ─── Params passed from RosBridge after loading ROS parameters ───────────────
struct SlamParams
{
    double filter_size_surf_min  = 0.5;
    double filter_size_map_min   = 0.5;
    double cube_len              = 200.0;
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

// ─── Snapshot: data RosBridge needs after each frontend tick ──────────────────
struct SlamSnapshot
{
    bool valid               = false;
    double lidar_end_time    = 0.0;

    PoseState pose;
    geometry_msgs::msg::Quaternion geoQuat;
    Eigen::Matrix<double, 23, 23>  cov_P = Eigen::Matrix<double, 23, 23>::Zero();

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZINormal>> feats_undistort;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZINormal>> feats_down_body;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZINormal>> laserCloudOri;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZINormal>> pcl_wait_pub;

    int  effct_feat_num  = 0;
    bool dense_pub_en    = false;
    bool scan_pub_en     = false;
};

// ─── Forward declare the impl class ──────────────────────────────────────────
class SlamCoreImpl;

// ─── SlamCore ─────────────────────────────────────────────────────────────────
class SlamCore
{
public:
    SlamCore();
    ~SlamCore();

    void init(const SlamParams& params);

    void pushLidar(std::shared_ptr<pcl::PointCloud<pcl::PointXYZINormal>> cloud,
                   double timestamp);
    void pushImu(const sensor_msgs::msg::Imu::SharedPtr msg);

    void spinFrontendOnce();
    SlamSnapshot getSnapshot() const;
    bool saveMap() const;

private:
    std::unique_ptr<SlamCoreImpl> impl_;
};
