#pragma once

#include <iostream>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>

#include <deque>
#include <mutex>
#include <vector>
#include <algorithm>

#include "core/types/pcl_types.hpp"
#include "core/types/deskew_types.hpp"




struct MeasureGroup
{
    // livox_ros_driver2::msg::CustomMsg::SharedPtr lidar_msg;
    LidarFrame lidar_frame;
    double frame_end_time =0.0;
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imus;
};


class SlamCore
{
public:
    SlamCore();
    ~SlamCore();

    void pushImuMsg(const sensor_msgs::msg::Imu::SharedPtr msg);
    void pushLidarFrame(const LidarFrame& frame );
    void spinFrontendOnce();
    bool saveMap();


    /*getter*/
    PointCloudXYZITConstPtr getUndistortedCloud() const;
    PointCloudXYZITConstPtr getDebugPreprocessCloud() const;
    double getLastProcessedFrameTime() const;
    PointCloudXYZITConstPtr getWorldFrameCloud() const; 
    PointCloudXYZITConstPtr getAccumulatedMapCloud() const;



private:
    double toSec(const builtin_interfaces::msg::Time& stamp) const ;

    /**
    spinFrontendOnce()
    -> syncMeasureGroup(meas)
        ├──processMeasure
            ├── isValidMeasure
            ├── initializeImu
            │   ├── accumulateImuInitStats
            │   ├── isImuInitReady
            │   └── finalizeImuInitStates
            (여기부터 통계적으로 imu초기값 설정 완료)
            └── runFrontend
            │   └──predictByImu
                    prepareImuSequence
                        ├─ updatePredRot
            │   └──undistortLidar
                        ├─ estimatePoseByScanToMap(meas)
                        └─ updateLocalMap(meas)
    */ 
    
    bool syncMeasureGroup(MeasureGroup& meas);

    bool hasSensorBuffer() const ;
    bool prepareLidarFrame(MeasureGroup& meas);
    bool hasEnoughImuForCurrentLidarFrame() const ;
    void collectImuForCurrentLidarFrame(MeasureGroup& meas);
    void consumeCurrentLidarFrameBuffer();
    void debugImuBufferBeforeCollect(const MeasureGroup& meas) const;
    /*

     */
    
    void processMeasure(const MeasureGroup& meas);
    bool isValidMeasure(const MeasureGroup& meas) const;
    void initializeImu(const MeasureGroup& meas) ;
    void accumulateImuInitStats(const MeasureGroup& meas);
    void finalizeImuInitStats();
    void runFrontend(const MeasureGroup& meas); //초기화 이후  frontend시작
    void predictByImu(const MeasureGroup& meas);
    std::vector<sensor_msgs::msg::Imu::SharedPtr> prepareImuSequence(const MeasureGroup& meas) const;
    void predictToLidarFrameEnd(const MeasureGroup& meas) ;
    void undistortLidar(const MeasureGroup& meas) ;

    void resetDeskewPoses(std::size_t expected_size);
    void updatePredictionTail(const MeasureGroup& meas);
    const  DeskewPose* findDeskewPose(double point_time) const;

    void logImuInitStatus();
    bool isValidImu(const MeasureGroup& meas) const;

    void pushDeskewBeginPose(const MeasureGroup& meas);
    void logImuCoverage(const MeasureGroup& meas) const;

    void predictOneImuInterval(
        const sensor_msgs::msg::Imu::SharedPtr& head,
        const sensor_msgs::msg::Imu::SharedPtr& tail,
        const MeasureGroup& meas
        );

    void updatePredRot(const Eigen::Vector3d& gyro_avg, double dt);
    void discardOldImuBeforeFrame(double frame_beg_time);

    /*prediction*/
    bool isPredictionStateNormal() const;
    void clearMap();

    /*poseEstimate*/
    void estimatePose(const MeasureGroup& meas);
    bool hasEnoughMapForRegistration() const;
    void initialPoseGuessFromPred();
    void buildRegistrationTarget();
    bool registerCurrentScanToMap();
    void transformCurrentScanToWorld();
    void updateLocalMap() ;
    
    PointCloudXYZITPtr downsampleCloud(const PointCloudXYZITConstPtr& input, float leaf_size) const;
    void updateCurrentPoseFromPrediction();
    bool hasMissedImuForCurrentLidarFrame() const;

    void updateAccumulatedMap() ;

    void updateLastAcceptedPose(const Eigen::Matrix4f& pose);
    bool isPoseJumpTooLarge(const Eigen::Matrix4f& pose) const;
    void updateTrackingReference(const Eigen::Matrix4f& pose);
    
private: 
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_buffer_;
    // std::deque<livox_ros_driver2::msg::CustomMsg::SharedPtr> lidar_buffer_;
    // std::deque<double> lidar_time_buffer_;
    std::deque<LidarFrame> lidar_frame_buffer_;

    std::mutex buffer_mutex_;
    

    double last_timestamp_imu_ = 0.0;
    double last_timestamp_lidar_ = 0.0;

    double lidar_end_time_ = 0.0;
    double lidar_mean_scantime_ = 0.1;
    int scan_num =0;
    bool lidar_pushed_ = false;

    int init_frame_count_ = 0; // 이번 lidar frame의 갯수
    int imu_count_in_frame_ =0;
    bool imu_initialized_ = false;

    Eigen::Vector3d mean_acc_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_gyr_ = Eigen::Vector3d::Zero();
    int init_imu_sample_count_ =0;
    

    sensor_msgs::msg::Imu::SharedPtr last_imu_;
    double last_lidar_end_time_ = 0.0;

    std::vector<DeskewPose> deskew_poses_;
    Eigen::Matrix3d pred_rot_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d pred_vel_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d pred_pos_ = Eigen::Vector3d::Zero();


    PointCloudXYZITPtr undistorted_cloud_;
    PointCloudXYZITPtr debug_preprocess_cloud_;
    double last_processed_frame_time_ = 0.0;

    /*prediction*/
    Eigen::Vector3d gravity_  =Eigen::Vector3d(0.0, 0.0, -9.81);
    Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();

    bool has_last_accepted_pose_ = false;
    Eigen::Matrix4f last_accepted_pose_ = Eigen::Matrix4f::Identity();

    double max_translation_per_frame_ = 1.0;
    double max_rotation_per_frame_rad_ = 30.0 * 3.14159265358979323846 / 180.0;


    /*poseEstimate*/
    Eigen::Matrix3d current_rot_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d current_pos_ = Eigen::Vector3d::Zero();
    PointCloudXYZITPtr registration_target_cloud_;
    PointCloudXYZITPtr world_frame_cloud_;
    PointCloudXYZITPtr accumulated_map_cloud_;

    Eigen::Matrix4f pose_guess_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f corrected_pose_ = Eigen::Matrix4f::Identity();

        /*registration*/
    PointCloudXYZITPtr local_map_cloud_;
    std::deque<PointCloudXYZITPtr> recent_world_frames_;

    std::size_t max_local_frames_ = 20;
    float max_registration_score_ = 1.5f;
    float source_voxel_leaf_size_ = 0.20f;
    float target_voxel_leaf_size_ = 0.30f;
    float last_registration_score_ = std::numeric_limits<float>::max();


    bool has_last_pred_for_guess_ = false;
    Eigen::Vector3d last_pred_pos_for_guess_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d last_pred_rot_for_guess_ = Eigen::Matrix3d::Identity();

    int seed_frame_count_ = 0;
    Eigen::Matrix4f tracking_pose_ = Eigen::Matrix4f::Identity();
    bool has_tracking_pose_ = false;
    bool use_gicp_correction_ = false;
};