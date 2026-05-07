

#include "core/slam_core.hpp"
#include <sensor_msgs/msg/imu.hpp>

SlamCore::SlamCore() 
{
    is_first_lidar_ = true;
    lidar_pushed_ = false;
}
 SlamCore::~SlamCore()
 {


 }


 void SlamCore::spinFrontendOnce() 
 {

    MeasureGroup meas;
    if(!syncMeasure(meas))
    {

    }
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

void SlamCore::pushImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    const double imu_time 
        = static_cast<double>(msg->header.stamp.sec) 
            + static_cast<double>(msg->header.stamp.nanosec)* 1e-9;

    debugImuDt(imu_time);

    std::lock_guard<std::mutex> lock(buffer_mutex_);
    imu_buffer_.push_back(msg);
    last_timestamp_imu_ = imu_time;

       std::cout
        << "[pushImu] imu_buffer_.size(): "
        << imu_buffer_.size()
        << std::endl;


}


 bool SlamCore::isValidBuffer()
 {
    if(lidar_frame_buffer_.empty() || imu_buffer_.empty())
    return false;

    

 }

 void SlamCore::debugImuDt(const double imu_time)
 {

    /*비교 대상 없으니 저장만*/
    if(!has_last_imu_time_) 
    {
     
      last_imu_time_ = imu_time;
      has_last_imu_time_ = true;
      return;
    }
    const double dt = imu_time - last_imu_time_;
    if (dt <= 0.0) {
        std::cout
            << "[debugImuDt] ERROR dt <= 0"
            << ", dt=" << dt
            << std::endl;
    }
    else {
        const double hz = 1.0 / dt;
        std::cout
            << "[debugImuDt]"
            << " dt=" << dt
            << " hz=" << hz
            << std::endl;
    }

    last_imu_time_ = imu_time;
 }
/*      push start*/


 /*syncMeasure*/

 bool SlamCore::syncMeasure(MeasureGroup& meas) 
 {
    if(!isValidBuffer()) return false;
    //버퍼에 아무것도 없을때는 준비가 안되었으니 취소

    /*algorithm*/
    if (!lidar_pushed_)

    return true;
 }

 /*     snycMeasure*/

