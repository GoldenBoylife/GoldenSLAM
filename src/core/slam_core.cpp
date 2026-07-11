#include "core/slam_core.hpp"
#include <pcl/filters/voxel_grid.h>
#include <cmath>

static Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v)
{
    Eigen::Matrix3d m;
    m << 0.0, -v.z(), v.y(),
         v.z(), 0.0, -v.x(),
        -v.y(), v.x(), 0.0;
    return m;
}

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
/*
    순서
        정지상태의 imu초기값 얻기
        raw값에서 undistorted_cloud 로 얻기
        cloud를 world frame으로 transform
        downsampled
        ikd-tree로 residual candidate 얻기
        estimatePoseCorrrection





*/
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
    //여기서 undistorted cloud 얻음.

    if(undistorted && !undistorted->empty()) 
    {
        snapshot.cloud_undistorted = std::make_shared<CloudT>(*undistorted);
    }
    else 
    {
        snapshot.cloud_undistorted = std::make_shared<CloudT>(*meas.lidar_frame.cloud);
    }



    CloudTPtr cloud_world = transformCloudBodyToWorld( snapshot.cloud_undistorted,snapshot.state);

    CloudTPtr cloud_world_down = voxelDownsample(cloud_world, 0.2);

    std::cout << "[SlamCore::runDeskew][voxel] " 
                << " world= " << cloud_world->size()
                << " down= " << cloud_world_down->size()
                << " leaf=0.2"
                << std::endl;

    // processIkdTree(cloud_world_down);


    IkdTreeProcessResult ikd_result = processIkdTree(cloud_world_down);
    
    CloudTPtr map_cloud_to_add = cloud_world_down;

    if (ikd_result.should_add_to_map)
    {
        PoseCorrectionResult correction = estimatePoseCorrection(
                ikd_result.update_residuals,
                snapshot.state
            );
        
        bool correction_accepted = false;
        CloudTPtr accepted_cloud_to_add;


        if (correction.valid)
        {
            /* v0 안전장치:
                    dx를 100% 다 적용하지 않고 일부만 적용해봄
                    처음에는 0.5정도가 안전
            */
            esekfom_api_.applyPoseCorrection(correction.dx);

            snapshot.state = esekfom_api_.getPoseState();

            CloudTPtr corrected_world =
                transformCloudBodyToWorld(
                    snapshot.cloud_undistorted,
                    snapshot.state);

            CloudTPtr corrected_world_down =
                voxelDownsample(corrected_world, 0.2);

            map_cloud_to_add = corrected_world_down;

            std::cout << "[SlamCore::correction][apply] "
                    << " valid=" << correction.valid
                    << " update_count=" << correction.update_count
                    << " rot_norm=" << correction.rot_norm
                    << " trans_norm=" << correction.trans_norm
                    << " dx=" << correction.dx.transpose()
                    << " corrected_down=" << corrected_world_down->size()
                    << std::endl;
        }

        ikd_tree_api_.addPoints(map_cloud_to_add);

        std::cout << "[SlamCore::processIkdTree][add] "
                << " new_map_size=" << ikd_tree_api_.size()
                << std::endl;

        updateDebugPredictedMap(map_cloud_to_add);
    }
    else
    {
        updateDebugPredictedMap(cloud_world_down);
    }

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




CloudTPtr SlamCore::voxelDownsample(const CloudTPtr& cloud, double leaf_size) const
{
    CloudTPtr filtered = std::make_shared<CloudT>();

    if(!cloud || cloud->empty())    return filtered;


    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(
        static_cast<float>(leaf_size),
        static_cast<float>(leaf_size),
        static_cast<float>(leaf_size)
    );


    voxel.filter(*filtered);

    filtered->width = static_cast<std::uint32_t>(filtered->points.size());
    filtered->height = 1;
    filtered->is_dense = false;

    return filtered;
}

/**
 * 새로운 스캔 cloud_world
 * 처음 맵이면 ikd-tree에서 build하고, 
 * 이미 map 이 있으면 현재 scan point 마다 nearest search 해본 뒤, 마지막에 현재 cloud를 map 에 추가하는 함수.
 * 
 * 순서
 *  방어코드(비어 있는지?)
 *  if (ikd_tree가 처음)
 *      ikd tree build() 후 return
 *  for(모든 점)
 *      nearestSearch()-> nearest point 알아내고 거리 제곱 알아냄.
 *      필터링1 (현scan point와 점 거리 1m 이하)
 *      필터링2 (현scan point와 plane 거리 0.2m이하
 * 
 *  residual 후보들 result return
  */
// void SlamCore::processIkdTree(const CloudTPtr& cloud_world) 
IkdTreeProcessResult SlamCore::processIkdTree(const CloudTPtr& cloud_world)
{
    IkdTreeProcessResult result;

    if (!cloud_world)
    {
        std::cout << "[SlamCore::processIkdTree][WARN] null cloud "
                  << " map_size=" << ikd_tree_api_.size()
                  << std::endl;
        return result;
    }

    if (cloud_world->empty())
    {
        std::cout << "[SlamCore::processIkdTree][WARN] empty cloud "
                  << " map_size=" << ikd_tree_api_.size()
                  << " input_size=0"
                  << std::endl;
        return result;
    }
    /*초기화 안되었으면 ikd-tree에서 build하고 retsurn */
    if (!ikd_tree_api_.isInitialized())
    {
        ikd_tree_api_.build(cloud_world);

        result.map_initialized_this_frame = true;
        result.should_add_to_map = false;

        std::cout << "[SlamCore::processIkdTree][Init] "
                  << " input_size=" << cloud_world->size()
                  << " map_size=" << ikd_tree_api_.size()
                  << std::endl;

        return result;
    }

    result.should_add_to_map = true;

    const double dist5_thresh = 1.0;
    const double residual_thresh = 0.2;

    double sum_dist5 = 0.0;
    int dist_count = 0;

    double sum_abs_residual = 0.0;
    int residual_count = 0;

    double sum_update_abs_residual = 0.0;
    int update_residual_count = 0;

    result.update_residuals.clear();
    result.update_residuals.reserve(cloud_world->size());

    /*현재 scan point에서 ikdt-ree를 search함.*/
    for (const auto& pt : cloud_world->points)
    {
        std::vector<PointT> nearest_points;
        std::vector<float> squared_distances;

        const bool ok =
            ikd_tree_api_.nearestSearch(
                pt,
                5,
                nearest_points,
                squared_distances);

        if (!ok || nearest_points.size() < 5 || squared_distances.size() < 5)
        {
            ++result.search_fail;
            continue;
        }
        /*여기부터는 이미 nearest_points, squared_distances가 존재함. */
        ++result.search_found;

        const double dist5 =
            std::sqrt(static_cast<double>(squared_distances[4]));

        sum_dist5 += dist5;
        ++dist_count;

        if (dist5 > result.max_dist5)
        {
            result.max_dist5 = dist5;
        }

        if (dist5 >= dist5_thresh)
        {
            ++result.distance_reject;
            continue;
        }

        ++result.residual_candidate;

        PlaneResidual residual_info;
        residual_info.dist5 = dist5;

        const bool residual_ok =
            computePointToPlaneResidual(
                pt,
                nearest_points,
                residual_info);

        if (!residual_ok)
        {
            ++result.plane_fail;
            continue;
        }

        ++result.plane_ok;

        sum_abs_residual += residual_info.abs_residual;
        ++residual_count;

        if (residual_info.abs_residual > result.max_abs_residual)
        {
            result.max_abs_residual = residual_info.abs_residual;
        }

        if (residual_info.abs_residual >= residual_thresh)
        {
            ++result.residual_reject;
            continue;
        }

        ++result.residual_update_candidate;

        sum_update_abs_residual += residual_info.abs_residual;
        ++update_residual_count;

        if (residual_info.abs_residual > result.max_update_abs_residual)
        {
            result.max_update_abs_residual = residual_info.abs_residual;
        }

        result.update_residuals.push_back(residual_info);
    }

    result.avg_dist5 =
        dist_count > 0
            ? sum_dist5 / static_cast<double>(dist_count)
            : 0.0;

    result.avg_abs_residual =
        residual_count > 0
            ? sum_abs_residual / static_cast<double>(residual_count)
            : 0.0;

    result.avg_update_abs_residual =
        update_residual_count > 0
            ? sum_update_abs_residual / static_cast<double>(update_residual_count)
            : 0.0;

    std::cout << "[SlamCore::processIkdTree][search] "
              << " scan_size=" << cloud_world->size()
              << " map_size=" << ikd_tree_api_.size()
              << " search_found=" << result.search_found
              << " search_fail=" << result.search_fail
              << " avg_dist5=" << result.avg_dist5
              << " max_dist5=" << result.max_dist5
              << " dist5_thresh=" << dist5_thresh
              << " residual_candidate=" << result.residual_candidate
              << " distance_reject=" << result.distance_reject
              << " plane_ok=" << result.plane_ok
              << " plane_fail=" << result.plane_fail
              << " avg_abs_residual=" << result.avg_abs_residual
              << " max_abs_residual=" << result.max_abs_residual
              << " residual_thresh=" << residual_thresh
              << " residual_update_candidate=" << result.residual_update_candidate
              << " residual_reject=" << result.residual_reject
              << " avg_update_abs_residual=" << result.avg_update_abs_residual
              << " max_update_abs_residual=" << result.max_update_abs_residual
              << std::endl;

    return result;
}


/*
    correction 후보 residual 이용해서 pose 보정량 dx를 계산함.
    1. point-to-plane residual이용해서 pose-only least squares correction 추정
*/
PoseCorrectionResult SlamCore::estimatePoseCorrection(
    const std::vector<PlaneResidual>& update_residuals,
    const PoseState& state) const
{
    PoseCorrectionResult result;
    result.update_count = static_cast<int>(update_residuals.size());

    if (update_residuals.size() < 50)
    {
        std::cout << "[SlamCore::correction][skip] "
                  << " reason=not_enough_residual "
                  << " update_count=" << update_residuals.size()
                  << std::endl;
        return result;
    }

    Eigen::Matrix<double, 6, 6> H =
        Eigen::Matrix<double, 6, 6>::Zero();

    Eigen::Matrix<double, 6, 1> b =
        Eigen::Matrix<double, 6, 1>::Zero();

    const Eigen::Vector3d pose_pos = state.pos;

    for (const auto& residual_info : update_residuals)
    {
        const Eigen::Vector3d point_world(
            residual_info.point_world.x,
            residual_info.point_world.y,
            residual_info.point_world.z);

        const Eigen::Vector3d lever = point_world - pose_pos;

        Eigen::Matrix<double, 1, 6> J;
        J.setZero();

        J.block<1, 3>(0, 0) =
            -residual_info.normal.transpose() * skewSymmetric(lever);

        J.block<1, 3>(0, 3) =
            residual_info.normal.transpose();

        const double r = residual_info.residual;

        H += J.transpose() * J;
        b += -J.transpose() * r;
    }

    Eigen::Matrix<double, 6, 6> H_damped = H;
    H_damped += 1e-6 * Eigen::Matrix<double, 6, 6>::Identity();

    Eigen::Matrix<double, 6, 1> dx =
        H_damped.ldlt().solve(b);

    if (!dx.allFinite())
    {
        std::cout << "[SlamCore::correction][skip] "
                  << " reason=dx_not_finite"
                  << std::endl;
        return result;
    }

    result.dx = dx;
    result.rot_norm = dx.head<3>().norm();
    result.trans_norm = dx.tail<3>().norm();

    result.valid =
        result.rot_norm < 0.05 &&
        result.trans_norm < 0.3;

    std::cout << "[SlamCore::correction][solve] "
              << " update_count=" << result.update_count
              << " rot_norm=" << result.rot_norm
              << " trans_norm=" << result.trans_norm
              << " valid=" << result.valid
              << " dx=" << result.dx.transpose()
              << std::endl;

    return result;
}
/*
    nearest point 5개로 local plane을 PCA 방식으로 추정하는 함수

    nearest map point 5개
    -> 평균점 계산
    -> 점들이 center 주변에서 어느 방향으로 퍼져 잇는지 계산 (covariance)
    -> 가장 덜 퍼진 방향을 palne normal로 선택(SelfAdjointEigenSolver)

    즉 이 함수는 평면 방정식의 기준점과 법선 벡터를 구하는 함수

    방어코드(NaN, Inf가 생겼으면 falsse)

*/
bool SlamCore::fitPlaneFromNearestPoints(
    const std::vector<PointT>& nearest_points,
    Eigen::Vector3d& plane_normal,
    Eigen::Vector3d& plane_center) const
{
    /*5개 구한거 맞는지?*/
    if(nearest_points.size() < 5) 
    {
        return false;
    }

    plane_center.setZero();

    /*plane_center값 구함*/
    for(const auto& p: nearest_points) 
    {
        plane_center += Eigen::Vector3d(p.x, p.y, p.z);
    }

    plane_center /= static_cast<double>(nearest_points.size());
    //5개로 나누어서 평균 위치를 구함.


    /*점들이 center 주변에서 어느 방향으로 퍼져 있는지 계산(cov)-> 가장 덜 퍼진 방향을 plane normal로 선택*/
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for(const auto& p : nearest_points)
    {
        const Eigen::Vector3d q(p.x, p.y, p.z);
        const Eigen::Vector3d d = q- plane_center;
        cov += d* d.transpose();
    }


    cov /= static_cast<double>(nearest_points.size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);

    if(solver.info() != Eigen::Success) 
    {
        return false;
    }


    /*
        가장 작은 eigenvalue에 대응하는 eigenvector가 local plane의 normal 방향이다.
    */
    plane_normal = solver.eigenvectors().col(0);
    //col(0): 가장 작은 eigenvalue의 egienvector , 점들이 가장적게 퍼진 방향이 바로 평면의 normal 벡터값임. 
    plane_normal.normalize();
    //normal값이 1이 되어야 residual도 실제 거리로 나오게됨.

    if(!plane_normal.allFinite())   return false;
    //계산 중 NaN이나 Inf가 생겼는지 확인하고, 이런 값 쓰면 안되니까 false 반환

    return true;
}

bool SlamCore::computePointToPlaneResidual(
    const PointT& point_world,
    const std::vector<PointT>& nearest_points,
    PlaneResidual& residual_info) const
{
    Eigen::Vector3d normal;
    Eigen::Vector3d center;

    const bool plane_ok = fitPlaneFromNearestPoints(nearest_points, normal, center);

    if(!plane_ok)   return false;


    const Eigen::Vector3d p(
        point_world.x,
        point_world.y,
        point_world.z
    );

    const double residual = normal.dot(p -center);

    residual_info.point_world = point_world;
    residual_info.normal = normal;
    residual_info.center = center;
    residual_info.residual = residual;
    residual_info.abs_residual =std::abs(residual);


    return true;
}
/*
    EKF state를 실제로 바꾸지 않음.
    만약 dx를 적용하면 pose가 어떻게 될지를 미리 계산하는 preview함수


*/
PostState SlamCore::applyPoseCorrectionToState(const PostState& state, const Eigen::Matrix<double,6,1>& dx, double scale) const
{
    PostState corrected_state = state;
    const Eigen::Vector3d dtheta = scale & dx.head<3>();
    const Eigen::Vector3d dt = scale * dx.tail<3>();

    corrected_state.pos = corrected_state.pos + dt;

    const double angle = dtheta.norm();

    if(angle > 1e-12) 
    {
        const Eigen::Vector3d axis = dtheta / angle;
        const Eigen::Quaterniond dq(Eigen::AngleAxisd(angle,axis));

        /*estimatePoseCorrection()의 Jacobian은  Left perturbation 기준이다.
            따라서 R-new = Exp(dtheta) * R 형태로 적용한다.
        */
       corrected_state.rot = (dq * corrected_state.rot).normalized();
    }
    return corrected_state;

}
