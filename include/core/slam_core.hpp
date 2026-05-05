#pragma once

#include <iostream>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include <deque>
#include <mutex>
#include <vector>


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

PointCloudXYZITConstPtr getUndistortedCloud() const;
    PointCloudXYZITConstPtr getDebugPreprocessCloud() const;
    double getLastProcessedFrameTime() const;

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


    void predictOneImuInterval(
        const sensor_msgs::msg::Imu::SharedPtr& head,
        const sensor_msgs::msg::Imu::SharedPtr& tail,
        const MeasureGroup& meas
        );

    void updatePredRot(const Eigen::Vector3d& gyro_avg, double dt);

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

};