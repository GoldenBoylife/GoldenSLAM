#include "platform/ros_bridge.hpp"
#include "core/slam_core.hpp"  // SlamCore 전체 정의 (ros_bridge.cpp 에서만 사용)

#include <omp.h>
#include <cmath>
#include <pcl_conversions/pcl_conversions.h>

#define PUBFRAME_PERIOD (20)

static inline double stamp_to_sec(const builtin_interfaces::msg::Time& t)
{
    return get_time_sec(t);
}
static inline rclcpp::Time sec_to_stamp(double ts)
{
    int32_t  sec     = static_cast<int32_t>(std::floor(ts));
    uint32_t nanosec = static_cast<uint32_t>((ts - std::floor(ts)) * 1e9);
    return rclcpp::Time(sec, nanosec);
}

// ─── ROS 런타임 ──────────────────────────────────────────────────────────────
void RosBridge::rosInit(int argc, char** argv)
{
    rclcpp::init(argc, argv);
}

void RosBridge::rosShutdown()
{
    if (rclcpp::ok()) rclcpp::shutdown();
}

void RosBridge::spin()
{
    rclcpp::spin(shared_from_this());
    rclcpp::shutdown();
}

// ─── constructor ─────────────────────────────────────────────────────────────
RosBridge::RosBridge(SlamCore* core)
    : rclcpp::Node("laser_mapping")
    , core_(core)
    , p_pre_(std::make_shared<Preprocess>())
{
    path_.header.stamp    = this->get_clock()->now();
    path_.header.frame_id = "camera_init";

    loadParameters();
    setupSubscribers();
    setupPublishers();
    setupTimers();
    setupServices();

    RCLCPP_INFO(this->get_logger(), "RosBridge init finished.");
}

// ─── loadParameters ──────────────────────────────────────────────────────────
void RosBridge::loadParameters()
{
    // ── declare ──────────────────────────────────────────────────────────────
    this->declare_parameter<bool>("publish.path_en",              true);
    this->declare_parameter<bool>("publish.effect_map_en",        false);
    this->declare_parameter<bool>("publish.map_en",               false);
    this->declare_parameter<bool>("publish.scan_publish_en",      true);
    this->declare_parameter<bool>("publish.dense_publish_en",     true);
    this->declare_parameter<bool>("publish.scan_bodyframe_pub_en",true);
    this->declare_parameter<int>("max_iteration",                 4);
    this->declare_parameter<std::string>("map_file_path",         "");
    this->declare_parameter<std::string>("common.lid_topic",      "/livox/lidar");
    this->declare_parameter<std::string>("common.imu_topic",      "/livox/imu");
    this->declare_parameter<bool>("common.time_sync_en",          false);
    this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
    this->declare_parameter<double>("filter_size_corner",         0.5);
    this->declare_parameter<double>("filter_size_surf",           0.5);
    this->declare_parameter<double>("filter_size_map",            0.5);
    this->declare_parameter<double>("cube_side_length",           200.0);
    this->declare_parameter<float>("mapping.det_range",           300.0f);
    this->declare_parameter<double>("mapping.fov_degree",         180.0);
    this->declare_parameter<double>("mapping.gyr_cov",            0.1);
    this->declare_parameter<double>("mapping.acc_cov",            0.1);
    this->declare_parameter<double>("mapping.b_gyr_cov",          0.0001);
    this->declare_parameter<double>("mapping.b_acc_cov",          0.0001);
    this->declare_parameter<double>("preprocess.blind",           0.01);
    this->declare_parameter<int>("preprocess.lidar_type",         AVIA);
    this->declare_parameter<int>("preprocess.scan_line",          16);
    this->declare_parameter<int>("preprocess.timestamp_unit",     US);
    this->declare_parameter<int>("preprocess.scan_rate",          10);
    this->declare_parameter<int>("point_filter_num",              2);
    this->declare_parameter<bool>("feature_extract_enable",       false);
    this->declare_parameter<bool>("runtime_pos_log_enable",       false);
    this->declare_parameter<bool>("mapping.extrinsic_est_en",     true);
    this->declare_parameter<bool>("pcd_save.pcd_save_en",         false);
    this->declare_parameter<int>("pcd_save.interval",             -1);
    this->declare_parameter<std::vector<double>>("mapping.extrinsic_T", std::vector<double>());
    this->declare_parameter<std::vector<double>>("mapping.extrinsic_R", std::vector<double>());

    // ── get publish flags ────────────────────────────────────────────────────
    this->get_parameter_or("publish.path_en",               path_en_,          true);
    this->get_parameter_or("publish.effect_map_en",         effect_pub_en_,    false);
    this->get_parameter_or("publish.map_en",                map_pub_en_,       false);
    this->get_parameter_or("publish.scan_publish_en",       scan_pub_en_,      true);
    this->get_parameter_or("publish.dense_publish_en",      dense_pub_en_,     true);
    this->get_parameter_or("publish.scan_bodyframe_pub_en", scan_body_pub_en_, true);

    // ── get topic names ───────────────────────────────────────────────────────
    std::string lid_topic, imu_topic;
    this->get_parameter_or<std::string>("common.lid_topic", lid_topic, "/livox/lidar");
    this->get_parameter_or<std::string>("common.imu_topic", imu_topic, "/livox/imu");

    // ── get preprocess params ─────────────────────────────────────────────────
    this->get_parameter_or("preprocess.blind",           p_pre_->blind,            0.01);
    this->get_parameter_or("preprocess.lidar_type",      p_pre_->lidar_type,       (int)AVIA);
    this->get_parameter_or("preprocess.scan_line",       p_pre_->N_SCANS,          16);
    this->get_parameter_or("preprocess.timestamp_unit",  p_pre_->time_unit,        (int)US);
    this->get_parameter_or("preprocess.scan_rate",       p_pre_->SCAN_RATE,        10);
    this->get_parameter_or("point_filter_num",           p_pre_->point_filter_num, 2);
    this->get_parameter_or("feature_extract_enable",     p_pre_->feature_enabled,  false);

    RCLCPP_INFO(this->get_logger(), "p_pre->lidar_type %d", p_pre_->lidar_type);

    // ── build SlamParams ──────────────────────────────────────────────────────
    SlamParams sp;
    this->get_parameter_or("filter_size_surf",                    sp.filter_size_surf_min,      0.5);
    this->get_parameter_or("filter_size_map",                     sp.filter_size_map_min,       0.5);
    this->get_parameter_or("cube_side_length",                    sp.box_len,                  200.0);
    this->get_parameter_or("mapping.det_range",                   sp.det_range,                 300.0f);
    this->get_parameter_or("mapping.fov_degree",                  sp.fov_deg,                   180.0);
    this->get_parameter_or("mapping.gyr_cov",                     sp.gyr_cov,                   0.1);
    this->get_parameter_or("mapping.acc_cov",                     sp.acc_cov,                   0.1);
    this->get_parameter_or("mapping.b_gyr_cov",                   sp.b_gyr_cov,                 0.0001);
    this->get_parameter_or("mapping.b_acc_cov",                   sp.b_acc_cov,                 0.0001);
    this->get_parameter_or("mapping.extrinsic_est_en",            sp.extrinsic_est_en,           true);
    this->get_parameter_or("max_iteration",                       sp.num_max_iterations,         4);
    this->get_parameter_or("common.time_sync_en",                 sp.time_sync_en,               false);
    this->get_parameter_or("common.time_offset_lidar_to_imu",    sp.time_diff_lidar_to_imu,     0.0);
    this->get_parameter_or("runtime_pos_log_enable",              sp.runtime_pos_log,            false);
    this->get_parameter_or("pcd_save.pcd_save_en",                sp.pcd_save_en,                false);
    this->get_parameter_or("pcd_save.interval",                   sp.pcd_save_interval,          -1);
    this->get_parameter_or<std::string>("map_file_path",          sp.map_file_path,              "");
    this->get_parameter_or<std::vector<double>>("mapping.extrinsic_T", sp.extrinT, std::vector<double>());
    this->get_parameter_or<std::vector<double>>("mapping.extrinsic_R", sp.extrinR, std::vector<double>());
    sp.root_dir = ROOT_DIR;

    pcd_save_en_   = sp.pcd_save_en;
    map_file_path_ = sp.map_file_path;
    lid_topic_     = lid_topic;
    imu_topic_     = imu_topic;

    core_->init(sp);
    //어?? 여기에서 init하네 이래도 되는구나. 
}

// ─── setupSubscribers ────────────────────────────────────────────────────────
void RosBridge::setupSubscribers()
{
    if (p_pre_->lidar_type == AVIA)
    {
        sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            lid_topic_, 20,
            std::bind(&RosBridge::onLivoxLidarCB, this, std::placeholders::_1));
    }
    else
    {
        sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lid_topic_, rclcpp::SensorDataQoS(),
            std::bind(&RosBridge::onStandardLidarCB, this, std::placeholders::_1));
    }

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, 10,
        std::bind(&RosBridge::onImuCB, this, std::placeholders::_1));
}

// ─── setupPublishers ─────────────────────────────────────────────────────────
void RosBridge::setupPublishers()
{
    pubLaserCloudFull_      = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered",      20);
    pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 20);
    pubLaserCloudEffect_    = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected",        20);
    pubLaserCloudMap_       = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map",             20);
    pubOdomAftMapped_       = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 20);
    pubPath_                = this->create_publisher<nav_msgs::msg::Path>("/path",        20);
    tf_broadcaster_         = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

// ─── setupTimers ─────────────────────────────────────────────────────────────
void RosBridge::setupTimers()
{
    auto period_ms     = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 100.0));
    auto map_period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0));

    timer_ = rclcpp::create_timer(this, this->get_clock(), period_ms,
                                  std::bind(&RosBridge::onFrontendTimer, this));
    map_pub_timer_ = rclcpp::create_timer(this, this->get_clock(), map_period_ms,
                                          std::bind(&RosBridge::onMapPublishTimer, this));
}

// ─── setupServices ───────────────────────────────────────────────────────────
void RosBridge::setupServices()
{
    map_save_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "map_save",
        std::bind(&RosBridge::onMapSaveCB, this, std::placeholders::_1, std::placeholders::_2));
}

// ─── onStandardLidarCB ───────────────────────────────────────────────────────
void RosBridge::onStandardLidarCB(sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
    double cur_time = stamp_to_sec(msg->header.stamp);

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre_->process(msg, ptr);

    core_->pushLidar(ptr, cur_time);
}

// ─── onLivoxLidarCB ──────────────────────────────────────────────────────────
void RosBridge::onLivoxLidarCB(livox_ros_driver2::msg::CustomMsg::UniquePtr msg)
{
    double cur_time = stamp_to_sec(msg->header.stamp);

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre_->process(msg, ptr);

    core_->pushLidar(ptr, cur_time);
}

// ─── onImuCB ─────────────────────────────────────────────────────────────────
void RosBridge::onImuCB(sensor_msgs::msg::Imu::UniquePtr msg_in)
{
    ImuData imu;
    imu.timestamp = stamp_to_sec(msg_in->header.stamp);
    imu.linear_acceleration << msg_in->linear_acceleration.x,
                               msg_in->linear_acceleration.y,
                               msg_in->linear_acceleration.z;
    imu.angular_velocity    << msg_in->angular_velocity.x,
                               msg_in->angular_velocity.y,
                               msg_in->angular_velocity.z;
    core_->pushImu(imu);
}

// ─── onFrontendTimer ─────────────────────────────────────────────────────────
void RosBridge::onFrontendTimer()
{
    core_->spinFrontendOnce();

    const auto snap = core_->getSnapshot();
    if (!snap.valid) return;

    publishOdometry(snap);
    if (path_en_)                             publishPath(snap);
    if (scan_pub_en_)                         publishFrameWorld(snap);
    if (scan_pub_en_ && scan_body_pub_en_)    publishFrameBody(snap);
    if (effect_pub_en_)                       publishEffectWorld(snap);
}

// ─── onMapPublishTimer ───────────────────────────────────────────────────────
void RosBridge::onMapPublishTimer()
{
    if (!map_pub_en_) return;
    const auto snap = core_->getSnapshot();
    if (!snap.valid) return;
    publishMap(snap);
}

// ─── onMapSaveCB ─────────────────────────────────────────────────────────────
void RosBridge::onMapSaveCB(std_srvs::srv::Trigger::Request::ConstSharedPtr /*req*/,
                             std_srvs::srv::Trigger::Response::SharedPtr      res)
{
    RCLCPP_INFO(this->get_logger(), "Saving map to %s...", map_file_path_.c_str());
    if (pcd_save_en_ && core_->saveMap())
    {
        res->success = true;
        res->message = "Map saved.";
    }
    else
    {
        res->success = false;
        res->message = "Map save disabled or no data.";
    }
}

// ─── setPoseStamp helper ──────────────────────────────────────────────────────
template<typename T>
void RosBridge::setPoseStamp(T& out, const SlamSnapshot& snap)
{
    out.pose.position.x    = snap.pose.pos(0);
    out.pose.position.y    = snap.pose.pos(1);
    out.pose.position.z    = snap.pose.pos(2);
    out.pose.orientation.x = snap.geoQuat.x;
    out.pose.orientation.y = snap.geoQuat.y;
    out.pose.orientation.z = snap.geoQuat.z;
    out.pose.orientation.w = snap.geoQuat.w;
}

// ─── publishOdometry ─────────────────────────────────────────────────────────
void RosBridge::publishOdometry(const SlamSnapshot& snap)
{
    odomAftMapped_.header.frame_id = "camera_init";
    odomAftMapped_.child_frame_id  = "body";
    odomAftMapped_.header.stamp    = sec_to_stamp(snap.lidar_end_time);
    setPoseStamp(odomAftMapped_.pose, snap);
    pubOdomAftMapped_->publish(odomAftMapped_);

    const auto& P = snap.cov_P;
    for (int i = 0; i < 6; i++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped_.pose.covariance[i * 6 + 0] = P(k, 3);
        odomAftMapped_.pose.covariance[i * 6 + 1] = P(k, 4);
        odomAftMapped_.pose.covariance[i * 6 + 2] = P(k, 5);
        odomAftMapped_.pose.covariance[i * 6 + 3] = P(k, 0);
        odomAftMapped_.pose.covariance[i * 6 + 4] = P(k, 1);
        odomAftMapped_.pose.covariance[i * 6 + 5] = P(k, 2);
    }

    geometry_msgs::msg::TransformStamped trans;
    trans.header.frame_id = "camera_init";
    trans.header.stamp    = odomAftMapped_.header.stamp;
    trans.child_frame_id  = "body";
    trans.transform.translation.x = odomAftMapped_.pose.pose.position.x;
    trans.transform.translation.y = odomAftMapped_.pose.pose.position.y;
    trans.transform.translation.z = odomAftMapped_.pose.pose.position.z;
    trans.transform.rotation.w    = odomAftMapped_.pose.pose.orientation.w;
    trans.transform.rotation.x    = odomAftMapped_.pose.pose.orientation.x;
    trans.transform.rotation.y    = odomAftMapped_.pose.pose.orientation.y;
    trans.transform.rotation.z    = odomAftMapped_.pose.pose.orientation.z;
    tf_broadcaster_->sendTransform(trans);
}

// ─── publishPath ─────────────────────────────────────────────────────────────
void RosBridge::publishPath(const SlamSnapshot& snap)
{
    setPoseStamp(msg_body_pose_, snap);
    msg_body_pose_.header.stamp    = sec_to_stamp(snap.lidar_end_time);
    msg_body_pose_.header.frame_id = "camera_init";

    path_frame_counter_++;
    if (path_frame_counter_ % 10 == 0)
    {
        path_.poses.push_back(msg_body_pose_);
        pubPath_->publish(path_);
    }
}

// ─── publishFrameWorld ───────────────────────────────────────────────────────
void RosBridge::publishFrameWorld(const SlamSnapshot& snap)
{
    if (!scan_pub_en_) return;

    PointCloudXYZI::Ptr laserCloudFullRes =
        dense_pub_en_ ? snap.feats_undistort : snap.feats_down_body;
    if (!laserCloudFullRes) return;

    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        const PointType& pi = laserCloudFullRes->points[i];
        PointType& po       = laserCloudWorld->points[i];
        V3D p_body(pi.x, pi.y, pi.z);
        V3D p_global(snap.pose.rot *
                     (snap.pose.offset_R_L_I * p_body + snap.pose.offset_T_L_I) +
                     snap.pose.pos);
        po.x = p_global(0); po.y = p_global(1); po.z = p_global(2);
        po.intensity = pi.intensity;
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
    laserCloudmsg.header.stamp    = sec_to_stamp(snap.lidar_end_time);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudFull_->publish(laserCloudmsg);
}

// ─── publishFrameBody ────────────────────────────────────────────────────────
void RosBridge::publishFrameBody(const SlamSnapshot& snap)
{
    if (!snap.feats_undistort) return;
    int size = snap.feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        const PointType& pi = snap.feats_undistort->points[i];
        PointType& po       = laserCloudIMUBody->points[i];
        V3D p_body_lidar(pi.x, pi.y, pi.z);
        V3D p_body_imu(snap.pose.offset_R_L_I * p_body_lidar + snap.pose.offset_T_L_I);
        po.x = p_body_imu(0); po.y = p_body_imu(1); po.z = p_body_imu(2);
        po.intensity = pi.intensity;
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp    = sec_to_stamp(snap.lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body_->publish(laserCloudmsg);
}

// ─── publishEffectWorld ───────────────────────────────────────────────────────
void RosBridge::publishEffectWorld(const SlamSnapshot& snap)
{
    if (!snap.laserCloudOri || snap.effct_feat_num <= 0) return;
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(snap.effct_feat_num, 1));

    for (int i = 0; i < snap.effct_feat_num; i++)
    {
        const PointType& pi = snap.laserCloudOri->points[i];
        PointType& po       = laserCloudWorld->points[i];
        V3D p_b(pi.x, pi.y, pi.z);
        V3D p_w(snap.pose.rot * (snap.pose.offset_R_L_I * p_b + snap.pose.offset_T_L_I) + snap.pose.pos);
        po.x = p_w(0); po.y = p_w(1); po.z = p_w(2);
        po.intensity = pi.intensity;
    }

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*laserCloudWorld, msg);
    msg.header.stamp    = sec_to_stamp(snap.lidar_end_time);
    msg.header.frame_id = "camera_init";
    pubLaserCloudEffect_->publish(msg);
}

// ─── publishMap ───────────────────────────────────────────────────────────────
void RosBridge::publishMap(const SlamSnapshot& snap)
{
    if (!snap.pcl_wait_pub) return;

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*snap.pcl_wait_pub, laserCloudmsg);
    laserCloudmsg.header.stamp    = sec_to_stamp(snap.lidar_end_time);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudMap_->publish(laserCloudmsg);
}
