#pragma once

#include <mutex>
#include <deque>

#include "core/types/slam_types.hpp"
#include "core/types/lidar_frame.hpp"

#include "core/api/esekfom_api.hpp"
#include "core/api/ikd_tree_api.hpp"

#include "core/types/lidar_frame.hpp"
#include "core/types/slam_types.hpp"

class SlamCore 
{
public: 
    SlamCore();
    ~SlamCore();


    void setParams(SlamParams sp);

    void spinFrontendOnce();

    void pushImu(const ImuData& imu);
    void pushLidarFrame(const LidarFrame& lidar_frame);
    /*syncMeasure*/
    bool syncMeasure(MeasureGroup& meas);
    /*      syncMeasure*/

private:
    static SlamCore* s_instance_;
    SlamParams sp_;

    EsekfomApi esekfom_api_;
    IkdTreeApi ikd_tree_api_;

    bool is_first_lidar_ ;


    /*입력 버퍼*/
    std::mutex                          mtx_buffer_;
    std::deque<double>                  time_buffer_;
    std::deque<LidarFrame>     lidar_frame_buffer_;
    std::deque<ImuData>                 imu_buffer_;
    bool lidar_frame_pushed_;

    LidarFrame current_lidar_frame_;
    double  last_imu_timestamp_;
    double last_frame_timestamp_;
    /*syncMeasure*/

};