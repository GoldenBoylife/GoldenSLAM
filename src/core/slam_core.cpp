#include "core/slam_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

#include <pcl/filters/voxel_grid.h>

namespace
{
const char* frontendUpdateModeToString(const FrontendUpdateMode mode)
{
    switch (mode)
    {
        case FrontendUpdateMode::PredictionOnly:
            return "PredictionOnly";

        case FrontendUpdateMode::ShadowOnly:
            return "ShadowOnly";

        case FrontendUpdateMode::ApplyCorrection:
            return "ApplyCorrection";

        default:
            return "Unknown";
    }
}
}

SlamCore::SlamCore() 
    : is_first_lidar_(true),
      lidar_frame_pushed_(false),
      frontend_frame_count_(0),
      map_insert_count_(0),
      R_body_lidar_(Eigen::Matrix3d::Identity()),
      t_body_lidar_(Eigen::Vector3d::Zero())
{

 
}
 SlamCore::~SlamCore()
 {


 }

void SlamCore::spinFrontendOnce()
{
    MeasureGroup meas;

    if (!syncMeasure(meas))
    {
        return;
    }

    ++frontend_frame_count_;

    State predicted_state = current_state_;

    ImuPropagatedPoseHistory imu_pose_history;

    imu_processor_.propagate(
        meas,
        predicted_state,
        imu_pose_history);

    if (!imu_processor_.isInitialized())
    {
        current_state_ = predicted_state;
        return;
    }

    auto cloud_deskewed =
        pointcloud_deskew_.deskew(
            meas.lidar_frame,
            imu_pose_history);

    if (!cloud_deskewed || cloud_deskewed->empty())
    {
        current_state_ = predicted_state;
        return;
    }

    auto cloud_downsampled =
        downsampleCloud(
            cloud_deskewed,
            MAP_VOXEL_SIZE);

    if (!cloud_downsampled || cloud_downsampled->empty())
    {
        current_state_ = predicted_state;
        return;
    }

    std::cout
        << "[DeskewFrameCheck]"
        << " frame=" << frontend_frame_count_
        << " lidar_beg=" << meas.lidar_frame.frame_beg_time
        << " lidar_end=" << meas.lidar_frame.frame_end_time
        << " input=" << cloud_deskewed->points.size()
        << " downsampled=" << cloud_downsampled->points.size()
        << " predicted_pos=("
        << predicted_state.position.x() << ", "
        << predicted_state.position.y() << ", "
        << predicted_state.position.z() << ")"
        << std::endl;

    auto cloud_world_predicted =
        transformCloudToWorld(
            cloud_downsampled,
            predicted_state);

    State corrected_state = predicted_state;
    IekfUpdateResult update_result;

    if (FRONTEND_UPDATE_MODE != FrontendUpdateMode::PredictionOnly)
    {
        update_result =
            runPoseOnlyIekfShadowUpdate(
                cloud_downsampled,
                cloud_world_predicted,
                predicted_state,
                corrected_state);
    }

    const bool enough_residuals =
        update_result.residual_count >= MIN_IEKF_RESIDUAL_COUNT;

    const bool correction_is_small =
        update_result.dx_rot_norm < MAX_IEKF_DX_ROT_NORM &&
        update_result.dx_pos_norm < MAX_IEKF_DX_POS_NORM;

    const bool residual_is_good =
        update_result.mean_abs_residual < MAX_IEKF_MEAN_ABS_RESIDUAL &&
        update_result.max_abs_residual < MAX_IEKF_MAX_ABS_RESIDUAL;

    const bool shadow_is_good =
        update_result.shadow_checked &&
        update_result.shadow_valid_kept &&
        update_result.shadow_mean_improved &&
        update_result.shadow_after_mean_abs < MAX_IEKF_MEAN_ABS_RESIDUAL &&
        update_result.shadow_after_max_abs < MAX_IEKF_MAX_ABS_RESIDUAL;

    const bool pass_update_gate =
        update_result.updated &&
        enough_residuals &&
        correction_is_small &&
        shadow_is_good;

    State frontend_state = predicted_state;
    bool use_corrected_state = false;

    if (FRONTEND_UPDATE_MODE == FrontendUpdateMode::ApplyCorrection &&
        pass_update_gate)
    {
        frontend_state = corrected_state;
        use_corrected_state = true;
    }

    /*
        중요:
        ShadowOnly에서는 corrected_state를 적용하지 않기 때문에,
        predicted_state 기준 cloud를 계속 map에 넣으면 map이 IMU drift 방향으로 망가진다.

        따라서 ShadowOnly에서는 초반 몇 프레임만 bootstrap용으로 map에 넣고,
        그 이후에는 map을 freeze한다.
    */
    const bool map_is_empty =
        ikd_tree_map_.empty();

    const bool is_prediction_only =
        FRONTEND_UPDATE_MODE == FrontendUpdateMode::PredictionOnly;

    const bool is_shadow_only =
        FRONTEND_UPDATE_MODE == FrontendUpdateMode::ShadowOnly;

    const bool is_apply_correction =
        FRONTEND_UPDATE_MODE == FrontendUpdateMode::ApplyCorrection;

    const bool shadow_bootstrap_map =
        is_shadow_only &&
        map_insert_count_ < SHADOW_ONLY_BOOTSTRAP_MAP_FRAMES;

    const bool apply_correction_bootstrap =
        is_apply_correction &&
        map_insert_count_ < SHADOW_ONLY_BOOTSTRAP_MAP_FRAMES;

    const bool should_insert_map =
        map_is_empty ||
        is_prediction_only ||
        shadow_bootstrap_map ||
        apply_correction_bootstrap ||
        (is_apply_correction && use_corrected_state);

    std::cout
        << "[FrontendState]"
        << " mode=" << frontendUpdateModeToString(FRONTEND_UPDATE_MODE)
        << " updated=" << update_result.updated
        << " use_corrected=" << use_corrected_state
        << " residual_count=" << update_result.residual_count
        << " enough_residuals=" << enough_residuals
        << " correction_small=" << correction_is_small
        << " residual_good=" << residual_is_good
        << " shadow_checked=" << update_result.shadow_checked
        << " shadow_valid_kept=" << update_result.shadow_valid_kept
        << " shadow_mean_improved=" << update_result.shadow_mean_improved
        << " shadow_mean_improved_enough=" << update_result.shadow_mean_improved_enough
        << " shadow_good=" << shadow_is_good
        << " mean_abs=" << update_result.mean_abs_residual
        << " max_abs=" << update_result.max_abs_residual
        << " dx_rot_norm=" << update_result.dx_rot_norm
        << " dx_pos_norm=" << update_result.dx_pos_norm
        << " shadow_after_mean=" << update_result.shadow_after_mean_abs
        << " shadow_after_max=" << update_result.shadow_after_max_abs
        << std::endl;

    std::cout
        << "[MapUpdate]"
        << " insert=" << should_insert_map
        << " map_empty=" << map_is_empty
        << " map_insert_count=" << map_insert_count_
        << " shadow_bootstrap=" << shadow_bootstrap_map
        << " use_corrected=" << use_corrected_state
        << std::endl;

    if (should_insert_map)
    {
        auto cloud_world =
            transformCloudToWorld(
                cloud_downsampled,
                frontend_state);

        ikd_tree_map_.insertCloud(cloud_world);
        ++map_insert_count_;
    }

    updateFrontendSnapshot(
        meas,
        frontend_state,
        ikd_tree_map_.getDisplayMap());

    current_state_ = frontend_state;
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
CloudTPtr SlamCore::transformCloudToWorld(
    const CloudTConstPtr& cloud_lidar,
    const State& state)
{
    auto cloud_world = std::make_shared<CloudT>();

    if (!cloud_lidar || cloud_lidar->empty())
    {
        return cloud_world;
    }

    cloud_world->points.reserve(cloud_lidar->points.size());

    const Eigen::Matrix3d R_world_body =
        state.rotation.toRotationMatrix();

    const Eigen::Vector3d t_world_body =
        state.position;

    for (const auto& point_lidar : cloud_lidar->points)
    {
        /*
            FAST-LIO2 계열에서 중요한 순서:

            1. LiDAR frame point
            2. Body/IMU frame point
            3. World frame point

            현재 extrinsic은 identity지만,
            구조는 반드시 이 순서를 가져야 한다.
        */
        const Eigen::Vector3d point_body =
            transformLidarPointToBodyEigen(point_lidar);

        const Eigen::Vector3d point_world =
            R_world_body * point_body + t_world_body;

        PointT output_point = point_lidar;

        output_point.x = static_cast<float>(point_world.x());
        output_point.y = static_cast<float>(point_world.y());
        output_point.z = static_cast<float>(point_world.z());

        cloud_world->points.push_back(output_point);
    }

    cloud_world->width =
        static_cast<std::uint32_t>(cloud_world->points.size());

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
IekfUpdateResult SlamCore::runPoseOnlyIekfShadowUpdate(
    const CloudTConstPtr& cloud_lidar,
    const CloudTConstPtr& cloud_world,
    const State& predicted_state,
    State& corrected_state)
{
    IekfUpdateResult update_result;
    corrected_state = predicted_state;

    /*
        before: predicted_state 기준 residual.
        shadow validation baseline이고, iter=0의 linearization point이기도 하다.
    */
    const auto before =
        buildResidualCandidates(
            cloud_lidar,
            cloud_world);

    std::cout
        << "[IekfResidualData]"
        << " query=" << before.query_count
        << " valid=" << before.valid_count
        << " search_fail=" << before.search_fail_count
        << " candidate_fail=" << before.candidate_fail_count
        << " mean_abs=" << before.mean_abs_residual
        << " max_abs=" << before.max_abs_residual
        << std::endl;

    if (before.residuals.empty())
    {
        return update_result;
    }

    /*
        iEKF iteration loop.
        iter=0 : before residuals 재사용 (추가 cloud transform 없음).
        iter=1+ : 직전 state_k 기준으로 cloud 재계산 후 residual 재생성.
        수렴 조건 없이 MAX_IEKF_ITER 횟수만큼 돌리고, updated=false면 조기 종료.
    */
    static constexpr int MAX_IEKF_ITER = 3;

    State state_k = predicted_state;
    IekfUpdateResult last_iter_result;
    bool any_updated = false;

    for (int iter = 0; iter < MAX_IEKF_ITER; ++iter)
    {
        const std::vector<ResidualCandidate>* current_residuals;
        ResidualCandidateBuildResult recomputed;

        if (iter == 0)
        {
            current_residuals = &before.residuals;
        }
        else
        {
            CloudTPtr cloud_k =
                transformCloudToWorld(cloud_lidar, state_k);

            recomputed =
                buildResidualCandidates(cloud_lidar, cloud_k);

            if (recomputed.residuals.empty())
            {
                std::cout
                    << "[IekfIter " << iter << "] residuals empty, stopping"
                    << std::endl;
                break;
            }
            current_residuals = &recomputed.residuals;
        }

        State state_next;
        const IekfUpdateResult iter_result =
            iekf_updater_.update(
                state_k,
                *current_residuals,
                state_next);

        std::cout
            << "[IekfIter " << iter << "]"
            << " updated=" << iter_result.updated
            << " dx_rot=" << iter_result.dx_rot_norm
            << " dx_pos=" << iter_result.dx_pos_norm
            << " mean_abs=" << iter_result.mean_abs_residual
            << std::endl;

        if (!iter_result.updated)
        {
            break;
        }

        last_iter_result = iter_result;
        state_k = state_next;
        any_updated = true;
    }

    if (!any_updated)
    {
        return update_result;
    }

    corrected_state = state_k;

    /*
        update_result 채우기.
        residual_count/mean/max: before 기준 (predicted_state에서의 초기 품질).
        dx_rot_norm/dx_pos_norm: predicted → corrected 전체 누적 보정량.
    */
    update_result.updated = true;
    update_result.residual_count = before.valid_count;
    update_result.mean_abs_residual = before.mean_abs_residual;
    update_result.max_abs_residual = before.max_abs_residual;
    update_result.dx = last_iter_result.dx;

    const Eigen::AngleAxisd total_daa(
        predicted_state.rotation.inverse() * corrected_state.rotation);
    update_result.dx_rot_norm = total_daa.angle();
    update_result.dx_pos_norm =
        (corrected_state.position - predicted_state.position).norm();

    /*
        Shadow validation.
        corrected_state cloud residual이 before 대비 개선됐는지 확인한다.
    */
    {
        auto cloud_world_corrected =
            transformCloudToWorld(
                cloud_lidar,
                corrected_state);

        const auto after =
            buildResidualCandidates(
                cloud_lidar,
                cloud_world_corrected);

        const bool valid_kept =
            after.valid_count >=
            static_cast<std::size_t>(
                static_cast<double>(before.valid_count) * 0.8);

        const bool mean_improved =
            after.mean_abs_residual <
            before.mean_abs_residual;

        const bool mean_improved_enough =
            after.mean_abs_residual <
            before.mean_abs_residual * 0.9;

        update_result.shadow_checked = true;
        update_result.shadow_valid_kept = valid_kept;
        update_result.shadow_mean_improved = mean_improved;
        update_result.shadow_mean_improved_enough = mean_improved_enough;

        update_result.shadow_before_valid = before.valid_count;
        update_result.shadow_after_valid = after.valid_count;

        update_result.shadow_before_mean_abs = before.mean_abs_residual;
        update_result.shadow_after_mean_abs = after.mean_abs_residual;

        update_result.shadow_before_max_abs = before.max_abs_residual;
        update_result.shadow_after_max_abs = after.max_abs_residual;

        std::cout
            << "[IekfShadowValidation]"
            << " before_valid=" << before.valid_count
            << " after_valid=" << after.valid_count
            << " before_mean=" << before.mean_abs_residual
            << " after_mean=" << after.mean_abs_residual
            << " before_max=" << before.max_abs_residual
            << " after_max=" << after.max_abs_residual
            << " valid_kept=" << valid_kept
            << " mean_improved=" << mean_improved
            << " mean_improved_enough=" << mean_improved_enough
            << std::endl;
    }

    return update_result;
}


 /*     plane_residual */


 /*iekf*/
ResidualCandidateBuildResult SlamCore::buildResidualCandidates(
    const CloudTConstPtr& cloud_lidar,
    const CloudTConstPtr& cloud_world)
{
    ResidualCandidateBuildResult result;

    if (ikd_tree_map_.empty())
    {
        return result;
    }

    if (!cloud_lidar || !cloud_world)
    {
        return result;
    }

    if (cloud_lidar->empty() || cloud_world->empty())
    {
        return result;
    }

    const std::size_t point_count =
        std::min(
            cloud_lidar->points.size(),
            cloud_world->points.size());

    if (point_count == 0)
    {
        return result;
    }

    double sum_abs_residual = 0.0;
    double max_abs_residual = 0.0;

    for (std::size_t i = 0; i < point_count; ++i)
    {
        /*
            cloud_lidar:
                deskew + downsample 이후의 LiDAR frame point

            query_point_body:
                LiDAR -> Body extrinsic을 거친 point

            query_point_world:
                predicted_state 또는 corrected_state로 world에 올린 point
        */
        const PointT& query_point_lidar =
            cloud_lidar->points[i];

        const PointT query_point_body =
            transformLidarPointToBodyPoint(query_point_lidar);

        const PointT& query_point_world =
            cloud_world->points[i];

        ++result.query_count;

        std::vector<PointT> nearest_points;
        std::vector<float> squared_distances;

        const bool search_ok =
            ikd_tree_map_.nearestSearch(
                query_point_world,
                5,
                nearest_points,
                squared_distances);

        if (!search_ok)
        {
            ++result.search_fail_count;
            continue;
        }

        /*
            FAST-LIO2식 correspondence에서 중요한 gate.
            query point 주변 map point들이 너무 멀면
            local plane으로 믿으면 안 된다.
        */
        if (squared_distances.empty() ||
            squared_distances.back() > MAX_NEAREST_POINT_SQ_DISTANCE)
        {
            ++result.candidate_fail_count;
            continue;
        }

        ResidualCandidate candidate;

        const bool candidate_ok =
            plane_estimator_.buildResidualCandidate(
                query_point_body,
                query_point_world,
                nearest_points,
                squared_distances,
                candidate);

        if (!candidate_ok || !candidate.valid)
        {
            ++result.candidate_fail_count;
            continue;
        }

        /*
            residual 자체가 너무 큰 후보는 update에 넣지 않는다.
            지금은 기존 IEKF gate와 같은 값을 사용한다.
        */
        if (candidate.abs_residual > MAX_IEKF_MAX_ABS_RESIDUAL)
        {
            ++result.candidate_fail_count;
            continue;
        }

        result.residuals.push_back(candidate);
        ++result.valid_count;

        sum_abs_residual += candidate.abs_residual;

        if (candidate.abs_residual > max_abs_residual)
        {
            max_abs_residual = candidate.abs_residual;
        }
    }

    if (result.valid_count > 0)
    {
        result.mean_abs_residual =
            sum_abs_residual / static_cast<double>(result.valid_count);

        result.max_abs_residual =
            max_abs_residual;
    }

    return result;
}

Eigen::Vector3d SlamCore::transformLidarPointToBodyEigen(
    const PointT& point_lidar) const
{
    const Eigen::Vector3d p_lidar(
        point_lidar.x,
        point_lidar.y,
        point_lidar.z);

    return R_body_lidar_ * p_lidar + t_body_lidar_;
}

PointT SlamCore::transformLidarPointToBodyPoint(
    const PointT& point_lidar) const
{
    const Eigen::Vector3d p_body =
        transformLidarPointToBodyEigen(point_lidar);

    PointT point_body = point_lidar;

    point_body.x = static_cast<float>(p_body.x());
    point_body.y = static_cast<float>(p_body.y());
    point_body.z = static_cast<float>(p_body.z());

    return point_body;
}

 /*     iekf*/