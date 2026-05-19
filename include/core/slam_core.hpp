#pragma once

#include <iostream>

#include <deque>
#include <mutex>

#include <sensor_msgs/msg/imu.hpp>


#include "core/slam_core.hpp"
#include "core/types/pcl_types.hpp"
#include "core/types/state.hpp"
#include "core/types/frontend_snapshot.hpp"

#include "core/imu_processor.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


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

    /*imu_propagate*/
    CloudTPtr transformCloudToWorld(const CloudTConstPtr& cloud, const State& state);
    void updateDebugMap(const CloudTConstPtr& cloud_world);
    void updateFrontendSnapshot(const MeasureGroup& meas,const State& predicted_state,const CloudTPtr& cloud_world_predicted);


    /*      imu_propagate*/
    

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
    CloudTPtr debug_map_;   

    /*      imu_propagate*/

};