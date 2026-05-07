

#include "core/slam_core.hpp"
#include <sensor_msgs/msg/imu.hpp>

SlamCore::SlamCore() 
{
    std::cout << "SlamCore started" << std::endl;
    is_first_lidar_ = true;
    lidar_pushed_ = false;
}
 SlamCore::~SlamCore()
 {


 }


 void SlamCore::spinFrontendOnce() 
 {
    std::cout << "[SlamCore] spinFrontendOnce " << std::endl;

    MeasureGroup meas;
    if(!syncMeasure(meas))
    {

    }
 }


 bool SlamCore::syncMeasure(MeasureGroup& meas) 
 {
    if(!isValidBuffer()) return false;
    //버퍼에 아무것도 없을때는 준비가 안되었으니 취소

    /*algorithm*/
    if (!lidar_pushed_)

    return true;
 }


 bool SlamCore::isValidBuffer()
 {
    if(lidar_frame_buffer_.empty() || imu_buffer_.empty())
    return false;

    

 }

 /*push*/
void SlamCore::pushLidarFrame(const LidarFrame& lidar_frame)
{
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    // imu_buffer_.push_back(msg);
    lidar_frame_buffer_.push_back(lidar_frame);
    std::cout << "lidar_buffer_.size(): "
        <<lidar_frame_buffer_.size() <<std::endl;





}
/*      push start*/
