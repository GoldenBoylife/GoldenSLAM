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
    std::size_t debug_map_frame_limit_ = 200;  // ori 20 , 한번에 보여주는 accumulated cloud?


};