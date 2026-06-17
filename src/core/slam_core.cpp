#include "core/slam_core.hpp"

SlamCore::SlamCore()
{

       std::cout << "[SlamCore::SlamCore] this=" << this
              << " esekfom_api_=" << &esekfom_api_
              << std::endl;

    is_first_lidar_  = true;
    is_first_measure_group_ = true;
    
    lidar_frame_pushed_ = false;

    first_lidar_time_ = 0.0;
    last_imu_timestamp_ = 0.0;
    last_frame_timestamp_ = 0.0;
}

SlamCore::~SlamCore()
{

}


void SlamCore::setParams(SlamParams sp)
{

        std::cout << "[SlamCore::setParams] this=" << this
              << " esekfom_api_=" << &esekfom_api_
              << std::endl;
    std::cout << "[SlamCore::setParams] called" << std::endl;

    sp_ = sp;
    esekfom_api_.setParams(sp_);
    ikd_tree_api_.setParams(sp_);
    std::cout << "[SlamCore::setParams] finished" << std::endl;

    
}
/*순서
    1. syncMeasure
    2. first scan 처리
    3. runDeskew()
    4. IMU propagation 입력 검증
    5. EsekfomApi propagation 연결



*/
void SlamCore::spinFrontendOnce()
{
    // std::cout << "111" << std::endl;
    MeasureGroup meas;
    if(!syncMeasure(meas)) return;
    //데이터 구조 묶기.


 
    const double lidar_beg_time = meas.lidar_frame.frame_beg_time;
    const double lidar_end_time = meas.lidar_frame.frame_end_time;

    /*첫 measure_group*/
    if(is_first_measure_group_) 
    {
        first_lidar_time_  = lidar_beg_time;
        is_first_measure_group_ = false;

        // std::cout << "[spinFrontendOnce] first measure group"
        //     << " first_lidar_time=" << first_lidar_time_
        //     << " imu_count=" << meas.imus.size()
        //     << std::endl;
        return;
    }
    /*TODO: runDeskew*/
    // std::cout << "[spinFrontendOnce] runDeskew entry"
    //           << " lidar_beg=" << lidar_beg_time
    //           << " lidar_end=" << lidar_end_time
    //           << " imu_count=" << meas.imus.size()
    //           << std::endl;
    runDeskew(meas);

}



void SlamCore::pushLidarFrame(const LidarFrame& lidar_frame)
{
    // std::cout << "lidar" <<std::endl;
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
    // std::cout << "Imu" <<std::endl;

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
    // std::cout << "syncMeasure" <<std::endl;

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


/*Deskew*/
//실제 pose구하는 알고리즘 파트
void SlamCore::runDeskew(const MeasureGroup& meas)
{

        std::cout << "[SlamCore::runDeskew] this=" << this
              << " esekfom_api_=" << &esekfom_api_
              << std::endl;
    imu_processor_.process(meas, esekfom_api_);


    //later
    //imu_processor_.undistort(meas, esekfom_api_);
    //feats_undistort_ = imu_processor_.getUndistortedCloud();
}
/*      Deskew*/
