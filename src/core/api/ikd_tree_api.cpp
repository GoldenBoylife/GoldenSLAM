#include "core/api/ikd_tree_api.hpp"

IkdTreeApi::IkdTreeApi()
{

}

void IkdTreeApi::setParams(const SlamParams& params)
{
    has_params_ = true;

}

// void IkdTreeApi::pushLidarFrame(const LidarFrame& frame)
// {
//     if(!has_params_) return;
//     lidar_frames_.push_back(frame);

//     std::cout << "lidar_frames_.size() : " <<lidar_frames_.size() << std::endl;
// }

bool IkdTreeApi::hasFrame() const
{
    // return !lidar_frames_.empty();

}

// LidarFrame IkdTreeApi::popFrame()
// {
//     LidarFrame frame = lidar_frames_.front();
//     lidar_frames_.pop_front();
//     return frame;
// }