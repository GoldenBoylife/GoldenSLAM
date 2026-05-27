#include "core/slam_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

#include <pcl/filters/voxel_grid.h>

static constexpr  double MAX_IEKF_DX_ROT_NORM = 0.01; //rad, 0.05 :  2.8도,  0.01 : 0.57도
static constexpr double MAX_IEKF_DX_POS_NORM = 0.05; //meter
static constexpr double MAX_IEKF_MEAN_ABS_RESIDUAL = 0.07;
static constexpr double MAX_IEKF_MAX_ABS_RESIDUAL = 0.25;
static constexpr double MIN_IEKF_VALID_RATIO = 0.80;


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

    if (!imu_processor_.isInitialized())
{
    current_state_ = predicted_state;
    return;
}


    auto cloud_deskewed = pointcloud_deskew_.deskew(meas.lidar_frame,imu_pose_history);
if (!cloud_deskewed || cloud_deskewed->empty())
{
    current_state_ = predicted_state;
    return;
}

    auto cloud_downsampled = downsampleCloud(cloud_deskewed, MAP_VOXEL_SIZE);

    size_t input_count = cloud_deskewed->size();
    size_t output_count = cloud_downsampled->size();
    
    // std::cout 
    //     << "[Downsample] voxel size = " << MAP_VOXEL_SIZE
    //     << "input = " << input_count 
    //     << " output = " << output_count 
    //     << 
    // std::endl;

    auto cloud_world_predicted  = transformCloudToWorld(cloud_downsampled, predicted_state);


    State corrected_state = predicted_state;

    // debugNearestSearch(cloud_world);
    const auto update_result = debugBuildResidualCandidates(cloud_downsampled,cloud_world_predicted,predicted_state, corrected_state);

    State frontend_state = predicted_state;
    bool use_corrected_state = false;

    std::cout
        << "[FrontendState]"
        << " updated=" << update_result.updated
        << " use_corrected=" << use_corrected_state
        << " residual_count=" << update_result.residual_count
        << " mean_abs=" << update_result.mean_abs_residual
        << " max_abs=" << update_result.max_abs_residual
        << " dx_rot_norm=" << update_result.dx_rot_norm
        << " dx_pos_norm=" << update_result.dx_pos_norm
        << std::endl;

    auto cloud_world =
        transformCloudToWorld(
            cloud_downsampled,
            frontend_state);

    ikd_tree_map_.insertCloud(cloud_world);

    updateFrontendSnapshot(
        meas,
        frontend_state,
        ikd_tree_map_.getDisplayMap());

    current_state_ = frontend_state;
    // /*rejected 된것은 map아 안넣기 위해서*/
    // static int rejected_insert_counter = 0;

    // const bool enough_residuals =
    //     update_result.residual_count >= 80;

    // const bool unstable_reject =
    //     update_result.updated && !use_corrected_state && enough_residuals;

    // const bool frontier_or_lost =
    //     !enough_residuals;

    // bool insert_map = false;

    // if (use_corrected_state)
    // {
    //     insert_map = true;
    //     rejected_insert_counter = 0;
    // }
    // else if (frontier_or_lost)
    // {
    //     // 기존 map과 대응이 부족한 상태.
    //     // 새 영역으로 들어가는 중일 수 있으므로 map을 완전히 막으면 안 됨.
    //     ++rejected_insert_counter;

    //     // 너무 많이 넣으면 다시 흐려지므로 몇 frame에 한 번만 seed로 넣는다.
    //     insert_map = (rejected_insert_counter % 3 == 0);
    // }
    // else if (unstable_reject)
    // {
    //     // 대응은 충분한데 correction이 너무 큼.
    //     // 이 경우는 map 오염 가능성이 높으므로 skip.
    //     insert_map = false;
    // }
    // else
    // {
    //     insert_map = true;
    // }

    // if (insert_map)
    // {
    //     ikd_tree_map_.insertCloud(cloud_world);
    // }
    // else
    // {
    //     std::cout
    //         << "[MapUpdate] skip insert"
    //         << " residuals=" << update_result.residual_count
    //         << " use_corrected=" << use_corrected_state
    //         << " dx_rot_norm=" << update_result.dx_rot_norm
    //         << " dx_pos_norm=" << update_result.dx_pos_norm
    //         << std::endl;
    // }

    //     // std::cout 
    //     //     << "[IkdTreeMap]  add= "
    //     //     << cloud_world->size()
    //     //     << " total = " << ikd_tree_map_.size()
    //     //     << std::endl;


    //     // accumulateDebugMap(cloud_world);
    //     //
    //     updateFrontendSnapshot(meas,frontend_state,ikd_tree_map_.getDisplayMap());
    //     //debug_map_ : 실제 cloud 데이터를 누적해서 들고 있는 저장소
    //     // FrontendSnapshot은 publish해야 할 최신 결과 묶음.


    //     /*나중에 여기서 EKF update*/

    //     current_state_ = frontend_state;
    //     //current_state_ 는 로봇의 누적되는 상태이므로 전역변수로 해야함. 

    //     //EKF없이 예측값을 현재 상태로 임시 사용

    
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

    // std::cout << "[IkdTreeMap::NearestSearch]"
    //         << " ok = " << search_ok
    //         << " found= " << nearest_points.size();

    // if(!squared_distances.empty()) 
    // {
    //     std::cout << " first_sq_dist = " << squared_distances.front();
    // }
    // std::cout << std::endl;

    EstimatedPlane plane;
    const bool plane_ok = plane_estimator_.estimate(nearest_points, plane);
    std::cout << "plane_ok= " << plane_ok << std::endl;

    if(!plane_ok|| !plane.valid)    return;

    const Eigen::Vector3d q(
        query_point.x,
        query_point.y,
        query_point.z
    );

    const double residual = plane.normal.dot(q) + plane.offset;



    std::cout
        << " residual=" << residual
        << " abs_residual=" << std::abs(residual)
        << " normal=("
        << plane.normal.x() << ", "
        << plane.normal.y() << ", "
        << plane.normal.z() << ")"
        << " offset=" << plane.offset
        << " eig_min=" << plane.smallest_eigenvalue
        << std::endl;
    
}
 /*     ikd-tree */


 /*plane_residual */
    /*현재 cloud에서 residual 후보들을 디버그용으로 만들어보고 통계만 출력하는 함수
        현재 scan point 몇개 고름
        ->기존 map에서 nearest search
        -> 주변 map point로 plane fitting
        -> point to plane residual 계산
        -> valid 후보 개수와 residual 평균 최대값 출력

        cloud_world point 하나 선택
        -> ikd_tree_map_에서 가까운 map point 5개 찾기
        -> 그 5개로 local plane 추정
        -> query point가 plane에서 얼마나 떨어졌는지 residual 계산
        -> 통계 출력

    */
IekfUpdateResult  SlamCore::debugBuildResidualCandidates(
    const CloudTConstPtr& cloud_body,
    const CloudTConstPtr& cloud_world,
    const State& predicted_state,
    State& corrected_state
)
{
    corrected_state  = predicted_state;
    IekfUpdateResult empty_result;

    if(ikd_tree_map_.empty()) return empty_result;
    if(!cloud_body || cloud_body->empty())    return empty_result ;
    if(!cloud_world || cloud_world->empty())    return empty_result ;

    constexpr int K_NEAREST = 5; //3개여도 되지만 5개여야 안정적임
    constexpr std::size_t MAX_QUERY_COUNT = 200;




    int query_count =0;
    std::size_t search_fail_count =0;
    std::size_t candidate_fail_count =0;
    std::size_t valid_count =0;

    double sum_abs_residual =  0.0;
    double max_abs_residual = 0.0;

    std::vector<ResidualCandidate> candidates;
    candidates.reserve(MAX_QUERY_COUNT);


    /*디버그 단계에서는 모든 point 검사하지 않고 일정 간격으로 샘플링하기 위해서*/
    // 50개마다 하나씩 뽑음. 즉, 10000개 중에서 200개만 query point로 사용함
    const std::size_t step =  std::max<std::size_t>(1, cloud_world->points.size() / MAX_QUERY_COUNT); 
    for(size_t i = 0; i< cloud_world->points.size() ; i+= step)
    {
        // const PointT& query_point = cloud_world->points[i];
        const PointT& query_point_body = cloud_body->points[i];
        const PointT& query_point_world = cloud_world->points[i];

        ++query_count;

        std::vector<PointT> nearest_points;
        std::vector<float> squared_distances;



        const bool search_ok = ikd_tree_map_.nearestSearch(query_point_world, K_NEAREST, nearest_points, squared_distances);

        if(!search_ok || nearest_points.size() <3)
        {
            ++search_fail_count;
            continue;
        }

        ResidualCandidate candidate;

        const bool candidate_ok = plane_estimator_.buildResidualCandidate(query_point_body, query_point_world, nearest_points,squared_distances, candidate);

        if(!candidate_ok || !candidate.valid)
        {
            ++candidate_fail_count;
            continue;
        }
        candidates.push_back(candidate);


        ++valid_count;
        sum_abs_residual += candidate.abs_residual;

        if (candidate.abs_residual > max_abs_residual)
        {
            max_abs_residual = candidate.abs_residual;
        }
    }

    const double mean_abs_residual =
        valid_count > 0
            ? sum_abs_residual / static_cast<double>(valid_count)
            : 0.0;

    std::cout
        << "[IekfResidualData]"
        << " query=" << query_count
        << " valid=" << valid_count
        << " search_fail=" << search_fail_count
        << " candidate_fail=" << candidate_fail_count
        << " mean_abs=" << mean_abs_residual
        << " max_abs=" << max_abs_residual
        << std::endl;
        
    /*iEKF에 cadidate residual 넣고 jacobian 결과값 받기*/
    
    const auto update_result = iekf_updater_.update(predicted_state, candidates, corrected_state);

    
    return update_result;


}


 /*     plane_residual */