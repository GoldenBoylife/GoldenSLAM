#include "core/slam_core.hpp"

SlamCore::SlamCore()
{
    is_first_lidar_  = true;
    lidar_frame_pushed_ = false;
    last_imu_timestamp_ = 0.0;
    last_frame_timestamp_ = 0.0;
}

SlamCore::~SlamCore()
{

}


void SlamCore::setParams(SlamParams sp)
{
    sp_ = sp;
    esekfom_api_.setParams(sp_);
    ikd_tree_api_.setParams(sp_);
    
}

void SlamCore::spinFrontendOnce()
{
    // std::cout << "111" << std::endl;
    MeasureGroup meas;
    if(!syncMeasure(meas)) return;



 
    const double lidar_beg = meas.lidar_frame.frame_beg_time;
    const double lidar_end = meas.lidar_frame.frame_end_time;

    std::cout << "[spinFrontendOnce] "
              << "lidar_beg=" << lidar_beg
              << " lidar_end=" << lidar_end
              << " lidar_dt=" << (lidar_end - lidar_beg)
              << " imu_count=" << meas.imus.size();

    if (!meas.imus.empty())
    {
        const double first_imu_time = meas.imus.front().timestamp;
        const double last_imu_time  = meas.imus.back().timestamp;

        std::cout << " first_imu=" << first_imu_time
                  << " last_imu=" << last_imu_time
                  << " first_diff=" << (first_imu_time - lidar_beg)
                  << " last_diff=" << (lidar_end - last_imu_time);
    }

    std::cout << std::endl;
}



void SlamCore::pushLidarFrame(const LidarFrame& lidar_frame)
{
    std::cout << "lidar" <<std::endl;
    std::lock_guard<std::mutex> lock(mtx_buffer_);

    if(!is_first_lidar_ && lidar_frame.frame_end_time < last_frame_timestamp_)
    {
        std::cerr << "lidar 역행 발생 \n";
        lidar_frame_buffer_.clear();
        // time_buffer_.clear();
        lidar_frame_pushed_ = false;
    }
    if(is_first_lidar_) is_first_lidar_ = false;



    last_frame_timestamp_ = lidar_frame.frame_end_time;

    lidar_frame_buffer_.push_back(lidar_frame);
}

void SlamCore::pushImu(const ImuData& imu) 
{
    std::lock_guard<std::mutex> lock(mtx_buffer_);
    if(imu.timestamp < last_imu_timestamp_)
    {
        std::cerr << "imu 역행 발생 \n";
        imu_buffer_.clear();
    }
    last_imu_timestamp_ = imu.timestamp;
    imu_buffer_.push_back(imu);


}

/*syncMeasure*/
bool SlamCore::syncMeasure(MeasureGroup& meas)
{
    std::lock_guard<std::mutex> lock(mtx_buffer_);
    if(lidar_frame_buffer_.empty() || imu_buffer_.empty())  return false;


    if(!lidar_frame_pushed_)
    {
        current_lidar_frame_ = lidar_frame_buffer_.front();
        lidar_frame_pushed_ = true;
    }
    /*imu 최근 시간이 현 Lidar_frame의 끝시간보다 더 빠르다면, 아직 덜들어온 상태임*/
    if(last_imu_timestamp_ < current_lidar_frame_.frame_end_time)
        return false;


    /*이번 measure 구성*/
    meas.lidar_frame  = current_lidar_frame_; //이번 프레임 결정
    meas.imus.clear();
    
    /*이 안에는 lidarframe 처음~끝 시간 사이의 imu들이 들어와야함. */
    //imu 구성
    while(!imu_buffer_.empty())
    {
        ImuData imu = imu_buffer_.front();
        const auto& imu_time = imu.timestamp;
        /*현frame시작보다 더 빠른 오래된 imu값들은 consume*/
        // if(imu_time < current_lidar_frame_.frame_beg_time)
        // {
        //     imu_buffer_.pop_front();
        //     continue;
        // }
        /*현frame끝보다 늦은 imu_time만나면 break*/
        if(imu_time >current_lidar_frame_.frame_end_time) break;

        meas.imus.push_back(imu);
        imu_buffer_.pop_front();
    }
    
    lidar_frame_buffer_.pop_front();
    lidar_frame_pushed_  = false;

    return true;
    //lidar_frame_pushed_ = false해야됨
}