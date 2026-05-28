#pragma once

#include <cstddef>
#include <deque>
#include <mutex>
#include <vector>
#include <Eigen/Dense>

#include <sensor_msgs/msg/imu.hpp>

#include "core/types/pcl_types.hpp"
#include "core/types/state.hpp"
#include "core/types/frontend_snapshot.hpp"
#include "core/imu_processor.hpp"
#include "core/pointcloud_deskew.hpp"
#include "core/ikd_tree_map.hpp"
#include "core/plane_estimator.hpp"
#include "core/iekf_updater.hpp"


constexpr double  MAX_DEBUG_MAP_POINTS = 3000000; //RViz 상 최대 점 갯수
constexpr float MAP_VOXEL_SIZE = 0.2;

static constexpr double MAX_IEKF_DX_ROT_NORM = 0.010; // rad, 약 0.57도
static constexpr double MAX_IEKF_DX_POS_NORM = 0.070;  // meter, 7cm

static constexpr double MAX_IEKF_MEAN_ABS_RESIDUAL = 0.08;
static constexpr double MAX_IEKF_MAX_ABS_RESIDUAL = 0.20;

static constexpr std::size_t MIN_IEKF_RESIDUAL_COUNT = 100;
static constexpr float MAX_NEAREST_POINT_DISTANCE = 1.0f; // meter
static constexpr float MAX_NEAREST_POINT_SQ_DISTANCE = MAX_NEAREST_POINT_DISTANCE * MAX_NEAREST_POINT_DISTANCE;



struct ResidualCandidateBuildResult
{
    std::vector<ResidualCandidate> residuals;

    std::size_t query_count = 0;
    std::size_t valid_count = 0;
    std::size_t search_fail_count = 0;
    std::size_t candidate_fail_count = 0;

    double mean_abs_residual = 0.0;
    double max_abs_residual = 0.0;
};

enum class FrontendUpdateMode
{
    PredictionOnly,
    ShadowOnly,
    ApplyCorrection
};

static constexpr FrontendUpdateMode FRONTEND_UPDATE_MODE = FrontendUpdateMode::ApplyCorrection;
static constexpr std::size_t SHADOW_ONLY_BOOTSTRAP_MAP_FRAMES = 10;

class SlamCore
{
public:
    SlamCore();
    ~SlamCore();
    void spinFrontendOnce();
    /*push*/

    void pushLidarFrame(const LidarFrame& frame);
    //const라서 가리키는 녀석은 바꿀수 없지만, 
    //가리키는 녀석의 .poinsts같은 건 편집 가능
    //예 cloud.reset() 같은 건 불가, cloud->points.clear()가능
    void pushImu(const sensor_msgs::msg::Imu::SharedPtr msg);
    /*      push*/

    FrontendSnapshot getFrontendSnapshot() const;


private: 
    /*syncMeasure*/
    bool syncMeasure(MeasureGroup& meas);
    bool isValidBuffer();
    void debugImuDt(const double imu_time);
    // bool isValidLidarHz();
    /*          syncMeasure*/

    /*imu_propagate + undistortion*/
    CloudTPtr transformCloudToWorld(const CloudTConstPtr& cloud,const State& state);

    // void accumulateDebugMap(const CloudTConstPtr& cloud_world);


    void updateFrontendSnapshot(
        const MeasureGroup& meas,
        const State& predicted_state,
        const CloudTPtr& map_cloud);

    /*      imu_propagate + undistortion*/
    /*ikd-tree*/
    CloudTPtr downsampleCloud(const CloudTConstPtr& cloud, float voxel_size) const;
    void debugNearestSearch(const CloudTConstPtr& cloud_world);
    /*      ikd-tree */
    /*plane_residual*/
    IekfUpdateResult runPoseOnlyIekfShadowUpdate(
        const CloudTConstPtr& cloud_body,
        const CloudTConstPtr& cloud_world,
        const State& predicted_state,
        State& corrected_state);
    

    /*      plane_residual*/

    /*iekf*/
    ResidualCandidateBuildResult buildResidualCandidates(
        const CloudTConstPtr& cloud_body,
        const CloudTConstPtr& cloud_world);

    Eigen::Vector3d transformLidarPointToBodyEigen(const PointT& point_lidar) const;
    PointT transformLidarPointToBodyPoint(const PointT& point_lidar) const;
    /*      iekf*/

private: 
    bool is_first_lidar_;
    bool lidar_frame_pushed_;


    
    /*syncMeasure*/
    
    std::mutex buffer_mutex_;
    std::deque<LidarFrame> lidar_frame_buffer_;
    std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer_;
    bool has_last_imu_time_ =false;
    double last_timestamp_imu_=0.0;
    double last_imu_time_ =0.0;
    std::size_t imu_count_  = 0;
    LidarFrame current_lidar_frame_;
        /*          syncMeasure*/

    /*imu_propagate*/
    

    ImuProcessor imu_processor_;
    State current_state_;

    mutable std::mutex snapshot_mutex_;
    FrontendSnapshot latest_frontend_snapshot_;
    // CloudTPtr debug_map_;
    /*      imu_propagate*/

    /*undistortion*/
    PointCloudDeskew pointcloud_deskew_;
    /*      undistortion*/

    /*ikd-tree*/
    IkdTreeMap ikd_tree_map_;

    /*      ikd-tree*/
    /*plane_residual*/
    PlaneEstimator plane_estimator_;
    /*      plane_residual*/

    /*iEFK*/
    IekfUpdater iekf_updater_;

    Eigen::Matrix3d R_body_lidar_;
    Eigen::Vector3d t_body_lidar_;


    std::size_t frontend_frame_count_;
    std::size_t map_insert_count_;
    /*      iEKF */
    
};