

#include "core/slam_core.hpp"
#include <sensor_msgs/msg/imu.hpp>

SlamCore::SlamCore() 
{
    is_first_lidar_ = true;
    lidar_frame_pushed_ = false;
}
 SlamCore::~SlamCore()
 {


 }


 void SlamCore::spinFrontendOnce() 
 {

    MeasureGroup meas; 
    //지역변수로 해야  다음 syncMeasure 호출때 유지되지 않음. 
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
   std::lock_guard<std::mutex> lock(buffer_mutex_);

   if(!isValidBuffer()) return false;

   /*아직 처리 대상으로 잡은 LiDAR frame이 없다면 "저장"*/
   if(!lidar_frame_pushed_)
   {
        current_lidar_frame_ = lidar_frame_buffer_.front();
        lidar_frame_pushed_ = true;

        std::cout
            << "[syncMeasure] latch lidar frame"
            << " beg=" << current_lidar_frame_.frame_beg_time
            << " end=" << current_lidar_frame_.frame_end_time
            << std::endl;
   }
    /*
        IMU가 lidar frame 끝시간까지 충분히 들어왔는지 확인
         imu 최근 시간이 현 lidar_frame의 end시간보다 더빠르다면, 아직 덜 채워진것!
    */
   if(last_timestamp_imu_ < current_lidar_frame_.frame_end_time)  
   {
        std::cout
        << "[syncMeasure] wait imu"
        << " imu=" << last_timestamp_imu_
        << " lidar_end=" << current_lidar_frame_.frame_end_time
        << std::endl;

        return false;
   }

   /*이제 measure가 구성됨*/
   meas.lidar_frame = current_lidar_frame_; //이번 프레임 결정
   meas.imus.clear();
   //deque은 
   while(!imu_buffer_.empty())
   {
    const auto& imu_msg = imu_buffer_.front();

    

    const double imu_time =
            static_cast<double>(imu_msg->header.stamp.sec)
            + static_cast<double>(imu_msg->header.stamp.nanosec) * 1e-9;
    /*현frame에 어울리지 않는 오래된 imu값들은 consume*/
    if(imu_time < current_lidar_frame_.frame_beg_time)
    {
        imu_buffer_.pop_front();
        continue;
    }

    if(imu_time > current_lidar_frame_.frame_end_time) break;

    /*ownership transfer, 이제 meas가 담당할게^^*/
    meas.imus.push_back(imu_msg);
    imu_buffer_.pop_front();
   }

   
   /*lidar frame consume */
   lidar_frame_buffer_.pop_front();
   lidar_frame_pushed_ = false;

      std::cout
        << "[syncMeasure] SUCCESS"
        << " imu count=" << meas.imus.size()
        << std::endl;

    return true;
 }


 bool SlamCore::isValidBuffer()
 {
    if(lidar_frame_buffer_.empty() || imu_buffer_.empty())
        return false;
    return true;

    

 }
 /*     snycMeasure*/

