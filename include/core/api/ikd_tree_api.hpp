#pragma once

#include <deque>

#include "core/types/slam_types.hpp"
// #include "core/types/lidar_frame.hpp"

class IkdTreeApi
{
public:
    IkdTreeApi();

    void setParams(const SlamParams& params);
    // void pushLidarFrame(const LidarFrame& frame);

    bool hasFrame() const;
    // LidarFrame popFrame();

private:
    bool has_params_ =false;
    // std::deque<LidarFrame> lidar_frames_;
};