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


    imu_processor_.process(meas, esekfom_api_);


    SlamSnapShot snapshot;
    snapshot.valid = true;
    snapshot.lidar_beg_time = meas.lidar_frame.frame_beg_time;
    snapshot.lidar_end_time = meas.lidar_frame.frame_end_time;

    snapshot.state = esekfom_api_.getPoseState();


    /* 
        1차 publish 검증
            아직 실제 undistortion은 하지 읂는다.
            raw와 deskewd같은 cloud를 pulish해서, publish pipline부터 검증한다.
    */
    //포인터를 새 cloud로 reset하면서 그 안에 meas 내용으로 초기화
    snapshot.cloud_raw =std::make_shared<CloudT> (*meas.lidar_frame.cloud);

    imu_processor_.undistort(meas, esekfom_api_);

    CloudTPtr undistorted = imu_processor_.getUndistortedCloud();

    if(undistorted && !undistorted->empty()) 
    {
        snapshot.cloud_undistorted = std::make_shared<CloudT>(*undistorted);
    }
    else 
    {
        snapshot.cloud_undistorted = std::make_shared<CloudT>(*meas.lidar_frame.cloud);
    }



    CloudTPtr cloud_world = transformCloudBodyToWorld( snapshot.cloud_undistorted,snapshot.state);
    updateDebugPredictedMap(cloud_world);

    if(debug_map_predicted_)
    {
        snapshot.cloud_map_predicted = std::make_shared<CloudT>(*debug_map_predicted_);

    }

    {
        std::lock_guard<std::mutex> lock(mtx_snapshot_);
        latest_snapshot_ = snapshot;
        has_new_snapshot_ = true;
    }


    //latera
    // imu_processor_.undistort(meas, esekfom_api_);
    // feats_undistort_ = imu_processor_.getUndistortedCloud();
}



bool SlamCore::popSnapshot(SlamSnapShot& snapshot)
{
    std::lock_guard<std::mutex> lock(mtx_snapshot_);


    if(!has_new_snapshot_) return false;

    snapshot = latest_snapshot_;
    has_new_snapshot_ = false;

    return snapshot.valid;
}


CloudTPtr SlamCore::transformCloudBodyToWorld(
    const CloudTPtr& cloud,
    const PoseState& state) const
{
    CloudTPtr cloud_world = std::make_shared<CloudT>();

    if(!cloud || cloud->empty())    return cloud_world;

    cloud_world->reserve(cloud->size());

    const M3D R_WI = state.rot.toRotationMatrix(); //state로 구한 World기준 IMU 자세
    const V3D p_WI = state.pos;                     // state로 구한 World기준 IMU 위치

    const M3D R_LI = state.offset_R_L_I;
    const V3D T_LI = state.offset_T_L_I;

    for(const auto& pt : cloud->points)
    {

        const V3D p_L(
            static_cast<double>(pt.x),
            static_cast<double>(pt.y),
            static_cast<double>(pt.z));
        
        /*LiDAR frame -> IMU frame*/
        const V3D p_I = R_LI * p_L + T_LI;


        /* IMU frame -> World frame*/
        const V3D p_W = R_WI * p_I + p_WI;

        PointT world_pt = pt;
        world_pt.x = static_cast<float>(p_W.x());
        world_pt.y = static_cast<float>(p_W.y());
        world_pt.z = static_cast<float>(p_W.z());

        cloud_world->push_back(world_pt);
    }

    cloud_world->width = static_cast<std::uint32_t>(cloud_world->points.size());
    cloud_world->height =1;
    cloud_world->is_dense = false;

    return cloud_world;

    

}



void SlamCore::updateDebugPredictedMap(const CloudTPtr& cloud_world)
{
    if (!cloud_world || cloud_world->empty())
    {
        return;
    }

    debug_world_cloud_frames_.push_back(cloud_world);

    while (debug_world_cloud_frames_.size() > debug_map_frame_limit_)
    {
        debug_world_cloud_frames_.pop_front();
    }

    debug_map_predicted_ = std::make_shared<CloudT>();

    std::size_t total_size = 0;
    for (const auto& cloud : debug_world_cloud_frames_)
    {
        if (cloud)
        {
            total_size += cloud->size();
        }
    }

    debug_map_predicted_->reserve(total_size);

    for (const auto& cloud : debug_world_cloud_frames_)
    {
        if (!cloud)
        {
            continue;
        }

        debug_map_predicted_->points.insert(
            debug_map_predicted_->points.end(),
            cloud->points.begin(),
            cloud->points.end());
    }

    debug_map_predicted_->width =
        static_cast<std::uint32_t>(debug_map_predicted_->points.size());

    debug_map_predicted_->height = 1;
    debug_map_predicted_->is_dense = false;
}
