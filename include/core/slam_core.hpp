#pragma once

#include <mutex>
#include <deque>

#include "core/types/slam_types.hpp"
#include "core/types/lidar_frame.hpp"

#include "core/api/esekfom_api.hpp"
#include "core/api/ikd_tree_api.hpp"

#include "core/types/lidar_frame.hpp"
#include "core/types/slam_types.hpp"

#include "core/imu_processor.hpp"

class SlamCore 
{
public:  
    SlamCore();
    ~SlamCore();


    void setParams(SlamParams sp);

    void spinFrontendOnce();

    void pushImu(const ImuData& imu);
    void pushLidarFrame(const LidarFrame& lidar_frame);

    bool syncMeasure(MeasureGroup& meas);
    bool popSnapshot(SlamSnapShot& snapshot);

public: //params

private: 
    void runDeskew(const MeasureGroup& meas);
    CloudTPtr transformCloudBodyToWorld(
        const CloudTPtr& cloud,
        const PoseState& state) const;
    void updateDebugPredictedMap(const CloudTPtr& cloud_world);


    CloudTPtr voxelDownsample(const CloudTPtr& cloud, double leaf_size) const;

    bool fitPlaneFromNearestPoints(
        const std::vector<PointT>& nearest_points,
        Eigen::Vector3d& plane_normal,
        Eigen::Vector3d& plane_center) const;
    // nearest point 5개 정도 이용해서 local plane추정하고 그 plane의 normal vector와 center point를 구하는 함수

    bool computePointToPlaneResidual(
        const PointT& point_world,
        const std::vector<PointT>& nearest_points,
        PlaneResidual& residual_info) const;
    // 현재 scan point와 nearest map points로 만든 local plane 사이의 거리, 즉 point-to-plane residual을 계산하는 함수
    

    IkdTreeProcessResult  processIkdTree(const CloudTPtr& cloud_world);
    //현재 scan point와 map point 사이의 point-to-plane residual 후보 수집

    PoseCorrectionResult estimatePoseCorrection(const std::vector<PlaneResidual>& update_residuals, const PoseState& state) const;
    //residual들을 이용해서 dx계산
    // dx = [회전 보정량 3개, 위치 보정량 3개]

    PostState applyPoseCorrectionToState(const PostState& state, const Eigen::Matrix<double,6,1>& dx, double scale) const;
    
private: //params
    static SlamCore* s_instance_;
    SlamParams sp_;

    EsekfomApi esekfom_api_;
    IkdTreeApi ikd_tree_api_;
    ImuProcessor imu_processor_;

    bool is_first_lidar_ ;
    bool is_first_measure_group_;
    double  first_lidar_time_;

    /*입력 버퍼*/
    std::mutex                          mtx_buffer_;
    std::deque<double>                  time_buffer_;
    std::deque<LidarFrame>     lidar_frame_buffer_;
    std::deque<ImuData>                 imu_buffer_;
    bool lidar_frame_pushed_;

    LidarFrame current_lidar_frame_;
    double  last_imu_timestamp_;
    double last_frame_timestamp_;
    

    SlamSnapShot latest_snapshot_;
    bool has_new_snapshot_ = false;
    std::mutex mtx_snapshot_;

    std::shared_ptr<CloudT> debug_map_predicted_;


    std::deque<CloudTPtr> debug_world_cloud_frames_;
    std::size_t debug_map_frame_limit_ = 150;  // ori 20 , 한번에 보여주는 accumulated cloud?


};