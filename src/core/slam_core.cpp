

#include "core/slam_core.hpp"
#include <sensor_msgs/msg/imu.hpp>

#include <pcl/filters/voxel_grid.h>


SlamCore::SlamCore() 
{
    is_first_lidar_ = true;
    lidar_frame_pushed_ = false;
    // debug_map_ = std::make_shared<CloudT>();

 
}
 SlamCore::~SlamCore()
 {


 }


 void SlamCore::spinFrontendOnce() 
 {

    MeasureGroup meas; 
    //지역변수로 해야  다음 syncMeasure 호출때 유지되지 않음. 
    if(!syncMeasure(meas))  return;

    State predicted_state = current_state_;
    //이번 frame에서만 쓰니까 지역변수

    ImuPropagatedPoseHistory imu_pose_history;

    imu_processor_.propagate(meas, predicted_state,imu_pose_history);
    //로봇의 누적되는 상태가 predicted_state로 들어간다. 
    //이 값은 결국 world 좌표계로 tf할때 쓰인다. 

    

    auto cloud_deskewed = pointcloud_deskew_.deskew(meas.lidar_frame,imu_pose_history);


    auto cloud_downsampled = downsampleCloud(cloud_deskewed, MAP_VOXEL_SIZE);

    size_t input_count = cloud_deskewed->size();
    size_t output_count = cloud_downsampled->size();
    
    // std::cout 
    //     << "[Downsample] voxel size = " << MAP_VOXEL_SIZE
    //     << "input = " << input_count 
    //     << " output = " << output_count 
    //     << 
    // std::endl;

    auto cloud_world = transformCloudToWorld(cloud_downsampled, predicted_state);


    debugNearestSearch(cloud_world);

    ikd_tree_map_.insertCloud(cloud_world);

    // std::cout 
    //     << "[IkdTreeMap]  add= "
    //     << cloud_world->size()
    //     << " total = " << ikd_tree_map_.size()
    //     << std::endl;


    // accumulateDebugMap(cloud_world);
    //
    updateFrontendSnapshot(meas,predicted_state,ikd_tree_map_.getDisplayMap());
    //debug_map_ : 실제 cloud 데이터를 누적해서 들고 있는 저장소
    // FrontendSnapshot은 publish해야 할 최신 결과 묶음.


    /*나중에 여기서 EKF update*/

    current_state_ = predicted_state;
    //current_state_ 는 로봇의 누적되는 상태이므로 전역변수로 해야함. 

    //EKF없이 예측값을 현재 상태로 임시 사용
    
 }

 /*push*/
void SlamCore::pushLidarFrame(const LidarFrame& lidar_frame)
{
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    // imu_buffer_.push_back(msg);
    lidar_frame_buffer_.push_back(lidar_frame);
    // std::cout << "lidar_buffer_.size(): " << lidar_frame_buffer_.size() <<std::endl;





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

    //    std::cout
    //     << "[pushImu] imu_buffer_.size(): "
    //     << imu_buffer_.size()
    //     << std::endl;


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
        // std::cout
        //     << "[debugImuDt]"
        //     << " dt=" << dt
        //     << " hz=" << hz
        //     << std::endl;
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

        // std::cout
        //     << "[syncMeasure] latch lidar frame"
        //     << " beg=" << current_lidar_frame_.frame_beg_time
        //     << " end=" << current_lidar_frame_.frame_end_time
        //     << std::endl;
   }
    /*
        IMU가 lidar frame 끝시간까지 충분히 들어왔는지 확인
         imu 최근 시간이 현 lidar_frame의 end시간보다 더빠르다면, 아직 덜 채워진것!
    */
   if(last_timestamp_imu_ < current_lidar_frame_.frame_end_time)  
   {
        // std::cout
        // << "[syncMeasure] wait imu"
        // << " imu=" << last_timestamp_imu_
        // << " lidar_end=" << current_lidar_frame_.frame_end_time
        // << std::endl;

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

    //   std::cout
    //     << "[syncMeasure] SUCCESS"
    //     << " imu count=" << meas.imus.size()
    //     << std::endl;

    return true;
 }


 bool SlamCore::isValidBuffer()
 {
    if(lidar_frame_buffer_.empty() || imu_buffer_.empty())
        return false;
    return true;

    

 }
 /*     snycMeasure*/


 /*imu_propagate*/
CloudTPtr SlamCore::transformCloudToWorld(const CloudTConstPtr& cloud, const State& state)
{
    auto cloud_world = std::make_shared<CloudT>();

    if(!cloud || cloud->empty()) return cloud_world;

    cloud_world->points.reserve(cloud->points.size());

    const Eigen::Matrix3d R = state.rotation.toRotationMatrix();
    const Eigen::Vector3d t = state.position;

    for(const auto& p  : cloud->points)
    {
        Eigen::Vector3d pb(p.x,p.y, p.z);
        Eigen::Vector3d pw = R* pb + t;

        PointT q  = p;
        q.x = static_cast<float>(pw.x());
        q.y = static_cast<float>(pw.y());
        q.z = static_cast<float>(pw.z());

        cloud_world->points.push_back(q);
    }
    cloud_world->width = static_cast<std::uint32_t>(cloud_world->points.size());
    cloud_world->height = 1;
    cloud_world->is_dense = false;

    return cloud_world;
}
void SlamCore::updateFrontendSnapshot(
    const MeasureGroup& meas,
    const State& state,
    const CloudTPtr& map_cloud)
{
    FrontendSnapshot snapshot;

    snapshot.valid = true;
    snapshot.stamp = meas.lidar_frame.frame_end_time;
    snapshot.state = state;

    if(map_cloud)
        snapshot.map_cloud = std::make_shared<CloudT>(*map_cloud);
    else 
        snapshot.map_cloud = std::make_shared<CloudT>();

    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    latest_frontend_snapshot_ = snapshot;
}
FrontendSnapshot SlamCore::getFrontendSnapshot() const
{
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    return latest_frontend_snapshot_;
}

/*누적한다.*/
// void SlamCore::accumulateDebugMap(const CloudTConstPtr& cloud_world)
// {
//     if (!cloud_world || cloud_world->empty())
//     {
//         return;
//     }

//     if (!debug_map_)
//     {
//         debug_map_ = std::make_shared<CloudT>();
//     }

//     *debug_map_ += *cloud_world;

//     constexpr std::size_t kMaxDebugMapPoints = 300000;

//     if (debug_map_->points.size() > kMaxDebugMapPoints)
//     {
//         const std::size_t remove_count =
//             debug_map_->points.size() - kMaxDebugMapPoints;

//         debug_map_->points.erase(
//             debug_map_->points.begin(),
//             debug_map_->points.begin() + static_cast<std::ptrdiff_t>(remove_count)
//         );
//     }

//     debug_map_->width = static_cast<std::uint32_t>(debug_map_->points.size());
//     debug_map_->height = 1;
//     debug_map_->is_dense = false;
// }

 /*     imu_propagate*/


 /*ikd-tree*/
CloudTPtr SlamCore::downsampleCloud(const CloudTConstPtr& cloud, const float voxel_size) const
{
    auto cloud_downsampled = std::make_shared<CloudT>();

    if(!cloud || cloud->empty())    return cloud_downsampled;

    if(voxel_size <= 0.0)
    {
        *cloud_downsampled = *cloud;
        return cloud_downsampled;
    }
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(voxel_size,voxel_size, voxel_size);

    voxel_filter.filter(*cloud_downsampled);

    return cloud_downsampled;
}


void SlamCore::debugNearestSearch(const CloudTConstPtr& cloud_world)
{
    if(ikd_tree_map_.empty()) return;
    if(!cloud_world || cloud_world->empty()) return;

    const PointT& query_point = cloud_world->points.front();

    std::vector<PointT> nearest_points;
    std::vector<float> squared_distances;

    const bool search_ok = ikd_tree_map_.nearestSearch(query_point,5,nearest_points,squared_distances);

    std::cout << "[IkdTreeMap::NearestSearch]"
            << " ok = " << search_ok
            << " found= " << nearest_points.size();

    if(!squared_distances.empty()) 
    {
        std::cout << " first_sq_dist = " << squared_distances.front();
    }
    std::cout << std::endl;
}
 /*     ikd-tree */