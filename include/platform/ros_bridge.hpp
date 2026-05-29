#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "core/slam_types.hpp"
#include "algorithm/lidar_preprocess.hpp"

class SlamCore;  // ros_bridge.hpp 는 SlamCore* 포인터만 사용 — 알고리즘 헤더 불필요

class RosBridge : public rclcpp::Node
{
public:
    // ROS 런타임 생명주기 — GoldenSlamApp 에서만 호출
    static void rosInit(int argc, char** argv);
    static void rosShutdown();

    explicit RosBridge(SlamCore* core);
    ~RosBridge() = default;

    void spin();

private:
    // ── setup ────────────────────────────────────────────────────────────────
    void loadParameters();
    void setupSubscribers();
    void setupPublishers();
    void setupTimers();
    void setupServices();

    // ── callbacks ────────────────────────────────────────────────────────────
    void onStandardLidarCB(sensor_msgs::msg::PointCloud2::UniquePtr msg);
    void onLivoxLidarCB(livox_ros_driver2::msg::CustomMsg::UniquePtr msg);
    void onImuCB(sensor_msgs::msg::Imu::UniquePtr msg_in);

    // ── timer callbacks ───────────────────────────────────────────────────────
    void onFrontendTimer();
    void onMapPublishTimer();

    // ── service callbacks ─────────────────────────────────────────────────────
    void onMapSaveCB(std_srvs::srv::Trigger::Request::ConstSharedPtr req,
                     std_srvs::srv::Trigger::Response::SharedPtr      res);

    // ── publish helpers ───────────────────────────────────────────────────────
    void publishOdometry(const SlamSnapshot& snap);
    void publishPath(const SlamSnapshot& snap);
    void publishFrameWorld(const SlamSnapshot& snap);
    void publishFrameBody(const SlamSnapshot& snap);
    void publishEffectWorld(const SlamSnapshot& snap);
    void publishMap(const SlamSnapshot& snap);

    template<typename T>
    void setPoseStamp(T& out, const SlamSnapshot& snap);

private:
    SlamCore* core_;

    // preprocessing (lives in RosBridge since callbacks use it)
    std::shared_ptr<Preprocess> p_pre_;

    // path accumulation
    nav_msgs::msg::Path path_;
    nav_msgs::msg::Odometry odomAftMapped_;
    geometry_msgs::msg::PoseStamped msg_body_pose_;
    int path_frame_counter_ = 0;

    // save params (needed by map_save service)
    bool        pcd_save_en_   = false;
    std::string map_file_path_ = "";

    // cached topic names
    std::string lid_topic_;
    std::string imu_topic_;

    // publish flags (from yaml)
    bool path_en_          = true;
    bool effect_pub_en_    = false;
    bool map_pub_en_       = false;
    bool scan_pub_en_      = true;
    bool dense_pub_en_     = true;
    bool scan_body_pub_en_ = true;

    // ── publishers ───────────────────────────────────────────────────────────
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr       pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr           pubPath_;

    // ── subscribers ──────────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr    sub_pcl_pc_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr            sub_imu_;

    // ── tf ───────────────────────────────────────────────────────────────────
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ── timers ───────────────────────────────────────────────────────────────
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;

    // ── services ─────────────────────────────────────────────────────────────
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;
};
