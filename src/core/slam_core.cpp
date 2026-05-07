

#include "core/slam_core.hpp"
#include <sensor_msgs/msg/imu.hpp>

SlamCore::SlamCore() 
{
    std::cout << "SlamCore started" << std::endl;
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
    if(isValidBuffer()) return false;
    return true;
 }


 bool SlamCore::isValidBuffer()
 {
    if(lidar_buffer_.empty() || imu_buffer_.empty())
    return false;
 }

 /*---push start---- */
//  void SlamCore::pushLidarBuffer(const )
 /*---push start---- */
