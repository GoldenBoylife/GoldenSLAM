#include "core/slam_core.hpp"

SlamCore::SlamCore() 
{
    std::cout << "SlamCore started" << std::endl;
}
SlamCore::~SlamCore()
{

}


double SlamCore::toSec(const builtin_interfaces::msg::Time& stamp) const
{
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

bool SlamCore::syncMeasureGroup(MeasureGroup& meas)
{
    /*
    순서
        mutex lock
        버퍼에 아무것도 없으면 준비가 안되어 있으니, return ;

        처음에는 lidar_pushed가 false라 올수 있고, 
        현재 처리 대상으로 잡아둔 lidar스캔이 아직 하나 없으면 골라 잡음. 

        if()
    */
   if(!hasSensorBuffer())               return false;
   if(!prepareLidarFrame(meas))         return false;
   if(!hasEnoughImuForCurrentLidarFrame())   return false;

   collectImuForCurrentLidarFrame(meas);
   consumeCurrentLidarFrame();

   return true;
}


bool SlamCore::hasSensorBuffer() const 
{
    // return !lidar_buffer_.empty() && !imu_buffer_.empty();
    return !lidar_frame_buffer_.empty() && !imu_buffer_.empty();
}

/*현재 처리 대상으로 잡아둔 LiDAR 프레임을 준비하고 끝시간까지 계산*/
bool SlamCore::prepareLidarFrame(MeasureGroup& meas)
{
    /*처음 일때 들어오기
     현재 처리 대상으로 잡아둔 LiDAr스캔이 아직 없으면 하나 골라서 대상으로 하기.
     lidar는 하나를 대상으로 잡아두었는데, 한 frame을 맞추려면 IMU값도 기다려야 함. 
    */
    if(lidar_frame_buffer_.empty())
        return false; 
    
    if(!lidar_pushed_)
    {

        meas.lidar_frame = lidar_frame_buffer_.front();   

        //너무 point size가 작으면(비정상)
        if(meas.lidar_frame.cloud->points.size() <= 1) 
        {
            lidar_end_time_ = meas.lidar_frame.frame_beg_time + lidar_mean_scantime_;
            //대략적으로 end_time을 넣음.
        }
        else 
        {
            /*사이즈 괜찮고, 마지막 점의 상대시간 값이 너무 작을 때(비정상), 평소의 평균값으로 떼우기*/
            const double scan_time = 
                meas.lidar_frame.cloud->points.back().relative_time;
                //마지막원소의 relative_time을 씀
            
            if(scan_time < 0.5 * lidar_mean_scantime_) 
            {
                lidar_end_time_ = meas.lidar_frame.frame_beg_time + lidar_mean_scantime_;
            }
            else 
            {
                /*정상: 사이즈 좋고, 마지막점 상대시간도 적당함*/
                ++scan_num;
                lidar_end_time_ = meas.lidar_frame.frame_beg_time + scan_time;
                lidar_mean_scantime_  +=  (scan_time - lidar_mean_scantime_) /scan_num;
                //평균적으로 한 스캔이 얼마나 걸리는지 학습
                std::cout << "lidar_mean_scantime_ = " << lidar_mean_scantime_ <<std::endl;
            }
        }
        meas.frame_end_time = lidar_end_time_;
        lidar_pushed_ = true;
    }
    return  true;
            
}
/*한 프레임이 준비완료되었는지 확인*/
bool SlamCore::hasEnoughImuForCurrentLidarFrame() const
{
    return last_timestamp_imu_ >= lidar_end_time_;
    //현 lidar frame 끝시간인 lidar_end_time_까지 imu가 들어와야 하니까. 
}

/*현 lidar_frame 끝시간 이하의 imu값들 모으기*/
//imu_buffer가 들어온 시점이 이번 frame인지 다음 frame인지 모르니까. 
//일단은 imu_buffer에 쌓아 놓고, 조건에 맞추어서 meas.imus에 넣는다. 
//전제: imu의 timestamp값이 오름차순 정렬되어 있다. 
// 현 상태에서 시간정렬 안된상태로 들어온다면, 이 코드는 무너진다.
// 따라서 시간 역전이 발생한다면, 그 버퍼는 clear해서 버려버리고 다시 시작한다. 
void SlamCore::collectImuForCurrentLidarFrame(MeasureGroup& meas)
{
    meas.imus.clear();

    while(!imu_buffer_.empty() )
    {
        const double  imu_time = toSec(imu_buffer_.front()->header.stamp);
        if(imu_time > lidar_end_time_) break;
        //만약 다음 frame꺼가 온다면, imu값을 buffer에 남겨둔다.그리고 

        meas.imus.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();

    }
}

void SlamCore::consumeCurrentLidarFrame()
{

}

// void SlamCore::pushLidarMsg(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
void SlamCore::pushLidarFrame(const LidarFrame& lidar_frame )
{
    std::cout << "pushLidarMsg" << std::endl;
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    /*bag파일문제나 센서,드라이버 문제로 시간 역전 현상을 방지하기 위해서 last_timestamp_lidar_로 항상 관리함*/
    if(lidar_frame.frame_beg_time < last_timestamp_lidar_)
    {
        std::cout << "[SlamCore] lidar loop back, clear lidar buffer" << std::endl;
        lidar_frame_buffer_.clear();
    }
    last_timestamp_lidar_ = lidar_frame.frame_beg_time;
    
    /*둘을 항상 짝맞춰 줘야 하기 때문에, 메시지버터와 시간 버퍼를 나눠서 관리*/
    // lidar_buffer_.push_back(msg);
    // lidar_time_buffer_.push_back(lidar_frame.frame_beg_time);
    lidar_frame_buffer_.push_back(lidar_frame);
}


void SlamCore::pushImuMsg(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::cout << "pushImuMsg" << std::endl;
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    const double timestamp = toSec(msg->header.stamp);

    /*시간 역순 방지하기 위해서 clear함*/
    if(timestamp < last_timestamp_imu_) 
    {
        std::cout << "[SlamCore] imu loop back, clear imu buffer" << std::endl;
        imu_buffer_.clear();
    }
    last_timestamp_imu_ = timestamp;
    imu_buffer_.push_back(msg);

}

void SlamCore::spinFrontendOnce()
{
    std::cout << "[SlamCore] spinFrontendOnce" << std::endl;

    MeasureGroup meas;

    if (!syncMeasureGroup(meas)) {
        std::cout << "[SlamCore] syncMeasureGroup false" << std::endl;
        return;
    }

        std::cout
        << "[Measure] "
        << "lidar pts = " << meas.lidar_frame.cloud->points.size()
        << ", imu count = " << meas.imus.size()
        << ", lidar beg = " << meas.lidar_frame.frame_beg_time
        << ", lidar end = " << meas.lidar_frame_end_time
        << std::endl;
}

bool SlamCore::saveMap()
{
    std::cout << "saveMap" << std::endl;
    return true;
}