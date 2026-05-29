#include <omp.h>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <pcl/io/pcd_io.h>

#include "core/slam_core.hpp"

#define INIT_TIME        (0.1)
#define LASER_POINT_COV  (0.001)

SlamCore* SlamCore::s_instance_ = nullptr;
//static 멤버 변수 정의

// ─────────────────────────────────────────────────────────────────────────────
// Layer 4: Measurement Model — static singleton callback
// ─────────────────────────────────────────────────────────────────────────────
void SlamCore::h_share_model_static(state_ikfom& s,
                                    esekfom::dyn_share_datastruct<double>& ekfom_data)
{
    s_instance_->h_share_model(s, ekfom_data);
}

// ─────────────────────────────────────────────────────────────────────────────
// Constructor / Destructor
// ─────────────────────────────────────────────────────────────────────────────
SlamCore::SlamCore()
    : featsFromMap_(new PointCloudXYZI())
    , feats_undistort_(new PointCloudXYZI())
    , feats_down_body_(new PointCloudXYZI())
    , feats_down_world_(new PointCloudXYZI())
    , normvec_(new PointCloudXYZI(100000, 1))
    , laserCloudOri_(new PointCloudXYZI(100000, 1))
    , corr_normvect_(new PointCloudXYZI(100000, 1))
    , _featsArray_(new PointCloudXYZI())
    , pcl_wait_pub_(new PointCloudXYZI())
    , XAxisPoint_body_(LIDAR_SP_LEN, 0.0f, 0.0f)
    , XAxisPoint_world_(LIDAR_SP_LEN, 0.0f, 0.0f)
    , position_last_(Zero3d)
    , Lidar_T_wrt_IMU_(Zero3d)
    , Lidar_R_wrt_IMU_(Eye3d)
{
    s_instance_ = this;
    memset(point_selected_surf_, true,    sizeof(point_selected_surf_));
    memset(res_last_,            -1000.0f, sizeof(res_last_));
}

SlamCore::~SlamCore()
{
    if (fp_) { fclose(fp_); fp_ = nullptr; }
    if (fout_pre_.is_open()) fout_pre_.close();
    if (fout_out_.is_open()) fout_out_.close();
}

// ─────────────────────────────────────────────────────────────────────────────
// init
// ─────────────────────────────────────────────────────────────────────────────
void SlamCore::init(const SlamParams& p)
{
    params_                   = p;
    filter_size_surf_min_     = p.filter_size_surf_min;
    filter_size_map_min_      = p.filter_size_map_min;
    DET_RANGE_                = p.det_range;
    extrinsic_est_en_         = p.extrinsic_est_en;
    NUM_MAX_ITERATIONS_       = p.num_max_iterations;
    time_sync_en_             = p.time_sync_en;
    time_diff_lidar_to_imu_   = p.time_diff_lidar_to_imu;
    runtime_pos_log_          = p.runtime_pos_log;
    pcd_save_en_              = p.pcd_save_en;
    map_file_path_            = p.map_file_path;
    root_dir_                 = p.root_dir;

    downSizeFilterSurf_.setLeafSize(filter_size_surf_min_, filter_size_surf_min_, filter_size_surf_min_);
    downSizeFilterMap_.setLeafSize(filter_size_map_min_,   filter_size_map_min_,  filter_size_map_min_);

    Lidar_T_wrt_IMU_ << VEC_FROM_ARRAY(p.extrinT);
    Lidar_R_wrt_IMU_ << MAT_FROM_ARRAY(p.extrinR);

    p_imu_.set_extrinsic(Lidar_T_wrt_IMU_, Lidar_R_wrt_IMU_);
    p_imu_.set_gyr_cov(V3D(p.gyr_cov,   p.gyr_cov,   p.gyr_cov));
    p_imu_.set_acc_cov(V3D(p.acc_cov,   p.acc_cov,   p.acc_cov));
    p_imu_.set_gyr_bias_cov(V3D(p.b_gyr_cov, p.b_gyr_cov, p.b_gyr_cov));
    p_imu_.set_acc_bias_cov(V3D(p.b_acc_cov, p.b_acc_cov, p.b_acc_cov));

    iekf_.init(NUM_MAX_ITERATIONS_, SlamCore::h_share_model_static);
    //h_share_model_static 이건 함수 포인터를 위한 인자임. 
    //iekf 안에서 이 SlamCore안에 있는 함수를 실행시키기 위해서 이렇게 꼬아져버림. 
    //함수 실행이 아니라, 함수 등록임. 나중에 update할때 이 함수 부르라는 뜻이됨. 
    // 핵심은 이건 콜백 등록한 것이고, iekf_.update(...)실행할때 

    std::string pos_log_dir = root_dir_ + "/Log/pos_log.txt";
    fp_ = fopen(pos_log_dir.c_str(), "w");
    fout_pre_.open(root_dir_ + "/Log/mat_pre.txt", std::ios::out);
    fout_out_.open(root_dir_ + "/Log/mat_out.txt", std::ios::out);
}

// ─────────────────────────────────────────────────────────────────────────────
// pushLidar / pushImu
// ─────────────────────────────────────────────────────────────────────────────
void SlamCore::pushLidar(PointCloudXYZI::Ptr cloud, double timestamp)
{
    std::lock_guard<std::mutex> lk(mtx_buffer_);
    scan_count_++;

    if (!is_first_lidar_ && timestamp < last_timestamp_lidar_)
    {
        std::cerr << "lidar loop back, clear buffer\n";
        lidar_buffer_.clear();
        time_buffer_.clear();
    }
    if (is_first_lidar_) is_first_lidar_ = false;

    last_timestamp_lidar_ = timestamp;

    if (!time_sync_en_ && abs(last_timestamp_imu_ - last_timestamp_lidar_) > 10.0 &&
        last_timestamp_imu_ > 0 && !imu_buffer_.empty() && !lidar_buffer_.empty())
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf\n",
               last_timestamp_imu_, last_timestamp_lidar_);
    }
    if (time_sync_en_ && !timediff_set_flg_ &&
        abs(last_timestamp_lidar_ - last_timestamp_imu_) > 1 && !imu_buffer_.empty())
    {
        timediff_set_flg_       = true;
        timediff_lidar_wrt_imu_ = last_timestamp_lidar_ + 0.1 - last_timestamp_imu_;
        printf("Self sync IMU and LiDAR, time diff is %.10lf\n", timediff_lidar_wrt_imu_);
    }

    lidar_buffer_.push_back(cloud);
    time_buffer_.push_back(timestamp);
}

void SlamCore::pushImu(const ImuData& imu_in)
{
    publish_count_++;

    ImuData imu   = imu_in;
    imu.timestamp -= time_diff_lidar_to_imu_;

    if (abs(timediff_lidar_wrt_imu_) > 0.1 && time_sync_en_)
        imu.timestamp = timediff_lidar_wrt_imu_ + imu_in.timestamp;

    std::lock_guard<std::mutex> lk(mtx_buffer_);
    if (imu.timestamp < last_timestamp_imu_)
    {
        std::cerr << "imu loop back, clear buffer\n";
        imu_buffer_.clear();
    }
    last_timestamp_imu_ = imu.timestamp;
    imu_buffer_.push_back(imu);
}

// ─────────────────────────────────────────────────────────────────────────────
// spinFrontendOnce — 100 Hz 타이머에서 호출
//   각 단계가 어느 층인지 한눈에 보이도록 오케스트레이터 역할만 담당
// ─────────────────────────────────────────────────────────────────────────────
void SlamCore::spinFrontendOnce()
{
    // ── 0. 입력 동기화 ────────────────────────────────────────────────────────
    if (!syncPackages(Measures_)) { updateSnapshot(false); return; }
    // syncPackages: LiDAR와 IMU를 한묶음으로 만들어주는 것. 


    if (flg_first_scan_) {
        first_lidar_time_       = Measures_.lidar_beg_time;
        p_imu_.first_lidar_time = first_lidar_time_;
        flg_first_scan_         = false;
        updateSnapshot(false);
        return;
    }

    double t0 = omp_get_wtime();
    match_time_ = kdtree_search_time_ = solve_time_ = 0.0;

    // ── Layer 2: Preprocess / Deskew ──────────────────────────────────────────
    //   IMU propagation → predicted state   +   point cloud undistortion
    runDeskew();
    if (feats_undistort_->empty()) { updateSnapshot(false); return; }

    flg_EKF_inited_ = (Measures_.lidar_beg_time - first_lidar_time_) < INIT_TIME ? false : true;

    // ── Layer 6a: Map FOV Segment ─────────────────────────────────────────────
    //   LiDAR 이동 → 로컬 맵 슬라이딩 윈도우 갱신
    lasermapFovSegment();

    // ── Layer 2b: Feature Downsample ──────────────────────────────────────────
    //   VoxelGrid : feats_undistort → feats_down_body
    downsampleFeatures(); //downsample

    // ── Layer 6b: Map Init (첫 프레임만) ──────────────────────────────────────
    if (!map_.isBuilt()) { buildInitialMap(); updateSnapshot(false); return; }
    if (feats_down_size_ < 5)  { updateSnapshot(false); return; }

    // ── Layer 5: Filter Engine ────────────────────────────────────────────────
    //   iEKF iterated update — 내부에서 Layer 4 h_share_model 반복 호출
    //   h_share_model: nearest search → plane fitting → residual → Jacobian
    double t_update_start = omp_get_wtime();
    double solve_H_time   = 0;
    runIEKFUpdate(solve_H_time);
    //iekf의 update가 됨. 
    
    double t_update_end   = omp_get_wtime();

    // ── Layer 6c: Map Incremental Update ──────────────────────────────────────
    //   보정된 포즈로 포인트를 world frame 변환 후 ikd-tree 에 추가
    accumulatePublishCloud();
    mapIncremental();

    double t3 = omp_get_wtime();

    // ── Layer 7: Output / Debug ───────────────────────────────────────────────
    if (runtime_pos_log_) dumpPerformanceLog(t0, t3, t_update_start, t_update_end, solve_H_time);
    updateSnapshot(true);
}

// ─────────────────────────────────────────────────────────────────────────────
// getSnapshot / saveMap
// ─────────────────────────────────────────────────────────────────────────────
SlamSnapshot SlamCore::getSnapshot() const
{
    std::lock_guard<std::mutex> lk(snapshot_mutex_);
    return latest_snapshot_;
}

bool SlamCore::saveMap() const
{
    if (!pcd_save_en_) return false;
    auto snap = getSnapshot();
    if (!snap.valid || !snap.pcl_wait_pub || snap.pcl_wait_pub->empty()) return false;
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(map_file_path_, *snap.pcl_wait_pub);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Layer 2: Preprocess / Deskew
// ─────────────────────────────────────────────────────────────────────────────
void SlamCore::runDeskew()
{
    p_imu_.Process(Measures_, iekf_, feats_undistort_);
    state_point_ = iekf_.get_x();
    pos_lid_     = state_point_.pos + state_point_.rot * state_point_.offset_T_L_I;
}

void SlamCore::downsampleFeatures()
{
    downSizeFilterSurf_.setInputCloud(feats_undistort_);
    downSizeFilterSurf_.filter(*feats_down_body_);
    feats_down_size_ = feats_down_body_->points.size();
}

// ─────────────────────────────────────────────────────────────────────────────
// Layer 5: Filter Engine
// ─────────────────────────────────────────────────────────────────────────────
void SlamCore::runIEKFUpdate(double& solve_H_time)
{
    normvec_->resize(feats_down_size_);
    feats_down_world_->resize(feats_down_size_);
    // pointSearchInd_surf_.resize(feats_down_size_);
    nearest_points_.resize(feats_down_size_);

    // pre-update 상태 로그 (mat_pre.txt)
    V3D ext_euler = SO3ToEuler(state_point_.offset_R_L_I);
    fout_pre_ << std::setw(20) << Measures_.lidar_beg_time - first_lidar_time_
              << " " << euler_cur_.transpose()
              << " " << state_point_.pos.transpose()
              << " " << ext_euler.transpose()
              << " " << state_point_.offset_T_L_I.transpose()
              << " " << state_point_.vel.transpose()
              << " " << state_point_.bg.transpose()
              << " " << state_point_.ba.transpose()
              << " " << state_point_.grav << "\n";

    iekf_.update(LASER_POINT_COV, solve_H_time);
    //이게 실행되면서, i

    // 보정된 상태 반영
    state_point_ = iekf_.get_x();
    euler_cur_   = SO3ToEuler(state_point_.rot);
    pos_lid_     = state_point_.pos + state_point_.rot * state_point_.offset_T_L_I;
}

// ─────────────────────────────────────────────────────────────────────────────
// Layer 4: Measurement Model
//   nearest search → plane fitting → residual → Jacobian
//   esekfom 이 update() 내부에서 반복 호출
// ─────────────────────────────────────────────────────────────────────────────

/*
    1. 초기화: 이번 update에서 쓸 유효 point와 normal정보 비움 
    2. downsample point를 world로 변환 
    //state_ikfom : IKfoM에 쓰이는 EKF state타입, world기준 좌표계, 
    //여기서 s는 고정도니 최종 state가 아니라 iEKF 반복 update 중 계속 바뀌면서 들어오는 현재 후보 state다. 
*/
void SlamCore::h_share_model(state_ikfom& s,
                              esekfom::dyn_share_datastruct<double>& ekfom_data)
{
    /*1. 초기화: 이번 update에서 쓸 유효 point와 normal정보 비움 */
    double match_start = omp_get_wtime();
    laserCloudOri_->clear();
    corr_normvect_->clear();
    total_residual_ = 0.0;

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for
#endif
        /*2. downsample point를 world로 변환 */
    for (int i = 0; i < feats_down_size_; i++)
    {
        PointType& point_body  = feats_down_body_->points[i];
        PointType& point_world = feats_down_world_->points[i];

        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
        //s는 현재 state임. 
        point_world.x = p_global(0); point_world.y = p_global(1);
        point_world.z = p_global(2); point_world.intensity = point_body.intensity;

        std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
        auto& points_near = nearest_points_[i];


        /*3. ikd-Tree에서 주변 map point 검색 */
        // 현재 LiDAR point 근처에 있는 map point 5개 검색
        // 효과가 있는 것들 true로 추림 그리고 저장함
        if (ekfom_data.converge)
        {
            map_.nearestSearch(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            // 현재 query point인 point_world 주변의 map point를 찾는다. 
            point_selected_surf_[i] = (points_near.size() >= (size_t)NUM_MATCH_POINTS) &&
                                       pointSearchSqDis[NUM_MATCH_POINTS - 1] <= 5.0f;
            //여기서 point_selected_surf_[i]가 true인 값만 유효한 point되고 EKF update에 쓰인다.
            //조건: 1. 주변 map point가 5개 이상,
            //     2. 마지막 nearest point 거리 제곱이 5.0이하여야함.
        }

        if (!point_selected_surf_[i]) continue;

        /*주변 map point들로 plane fitting*/
        // 주변점 5개가 plane으로 잘 맞아야함.
        VF(4) pabcd;
        point_selected_surf_[i] = false;

        //esti_plane ()  : plane식이 ax+by+cz+d = 0인데 a,b,c,d 구함
        //주변 map point들인 nearest poiont들로 하나의 평면을 만듦)
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            /*좋은 대응점만 찾기*/
            //여기서 살아남은 것만 EKF update에 씀
            float pd2 = pabcd(0)*point_world.x + pabcd(1)*point_world.y + pabcd(2)*point_world.z + pabcd(3);
            //point to plane residual 계산
            //pd2가 작으면 map plane 위에 잘 올라감., 크면 멀어진것임. 
            float s   = 1.0f - 0.9f * fabs(pd2) / sqrt(p_body.norm());

            /*residual이 충분히 작아야함 -> effective point*/
            if (s > 0.9f)
            {
                point_selected_surf_[i]          = true;
                normvec_->points[i].x             = pabcd(0);
                normvec_->points[i].y             = pabcd(1);
                normvec_->points[i].z             = pabcd(2);
                normvec_->points[i].intensity     = pd2;
                res_last_[i]                      = fabs(pd2);

            }
        }
    }

    /*평면이 충분히 좋다면, 그 point를 effective feature로 채택*/
    //point_selected_surf_ 가 true 인것들만 모아옴
    //선택된 point와 normal을 압축해서 저장
    //h_share_model()여기서 residual과 Jacobian을 IKFoM으로 넘긴다.
    //estimate를 해서 plane fitting함
    effct_feat_num_ = 0;
    for (int i = 0; i < feats_down_size_; i++)
    {
        if (point_selected_surf_[i])
        {
            laserCloudOri_->points[effct_feat_num_] = feats_down_body_->points[i]; 
            //EKF update에 실제로 사용할 유효 LiDAR point 목록
            //world기준이 아니라 body 기준인데, 이래야 jcobian만들때 현재 상태로 다시 미분해야 하기  때문
            //feats_down_body_ 전체 후보 point에서 좋은 것만 선택해서 laserCloudOri로 저장하고 EKF에도 쓴다. 

            corr_normvect_->points[effct_feat_num_] = normvec_->points[i];
            //laserCloudOri_에 들어간 point의 map plane 정보를 여기 담음. 


            total_residual_ += res_last_[i];
            effct_feat_num_++;
        }
    }

    if (effct_feat_num_ < 1)
    {
        ekfom_data.valid = false;
        std::cerr << "No Effective Points!\n";
        return;
    }

    res_mean_last_ = total_residual_ / effct_feat_num_;
    match_time_   += omp_get_wtime() - match_start;
    double solve_start = omp_get_wtime();

    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num_, 12);
    ekfom_data.h.resize(effct_feat_num_);

    for (int i = 0; i < effct_feat_num_; i++)
    {
        const PointType& laser_p = laserCloudOri_->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        //Jacobian과 residual 생성 

        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        const PointType& norm_p = corr_normvect_->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);
        // 해당 point의 plane normal 꺼냄

        V3D C(s.rot.conjugate() * norm_vec);
        V3D A(point_crossmat * C);

        if (extrinsic_est_en_)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
            ekfom_data.h_x.block<1,12>(i,0) << norm_p.x, norm_p.y, norm_p.z,
                VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1,12>(i,0) << norm_p.x, norm_p.y, norm_p.z,
                VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }
        ekfom_data.h(i) = -norm_p.intensity;
        //pd2를 EKF measurement residual로 씀
    }
    solve_time_ += omp_get_wtime() - solve_start;
}

// ─────────────────────────────────────────────────────────────────────────────
// Layer 6: Map Manager
// ─────────────────────────────────────────────────────────────────────────────
void SlamCore::buildInitialMap()
{
    if (feats_down_size_ > 5)
    {
        map_.setDownsampleParam(filter_size_map_min_);
        feats_down_world_->resize(feats_down_size_);
        for (int i = 0; i < feats_down_size_; i++)
            pointBodyToWorld(&(feats_down_body_->points[i]), &(feats_down_world_->points[i]));
        map_.build(feats_down_world_->points);
    }
}

/* 
설명 : 
1. LiDAR 현재 위치가 local map box의 가장자리에 가까워지면, 
2. local map  box를 로봇 쪽으로 이동
3. 새 box 범위 밖으로 밀려난 map point들을 ikd-Tree에서 삭제함
*/

//local map box 삭제
void SlamCore::lasermapFovSegment()
{
    cub_need_rm_.clear();  //삭제로 등록한 것들 소거!
    kdtree_delete_counter_ = 0;
    kdtree_delete_time_    = 0.0;

    {
        PointType pb, pw;
        pb.x = XAxisPoint_body_.x(); pb.y = XAxisPoint_body_.y(); pb.z = XAxisPoint_body_.z();
        pb.intensity = 0;
        pointBodyToWorld(&pb, &pw);
        XAxisPoint_world_.x() = pw.x; XAxisPoint_world_.y() = pw.y; XAxisPoint_world_.z() = pw.z;
    }

    V3D pos_LiD = pos_lid_; 
    /*처음 local map 생성 후*/
    if (!localmap_initialized_)
    {
        for (int i = 0; i < 3; i++)
        {
            local_map_points_.vertex_min[i] = pos_LiD(i) - params_.box_len / 2.0;
            local_map_points_.vertex_max[i] = pos_LiD(i) + params_.box_len / 2.0;  
            // box의 크기는 한변 길이가 200m
            //local_map_points_ 는 로봇 주변 box의  최대,최소임
        }
        localmap_initialized_ = true;
        return;
    }

    /*LiDAR위치가 local map box 가장자리에 가까운지 검사 */
    constexpr float MOV_THRESHOLD = 1.5f; //local map 이동판단 기준
    float dist_to_map_edge[3][2];
    bool  need_move = false;
    for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - local_map_points_.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - local_map_points_.vertex_max[i]);
        //LiDAR의 위치값과 local map box의 min,max와의 거리를 계산함
        //[0]에는 box의 min값, [1]에는 box의 max값

        /*가까워 지면 이동해야지~*/
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE_ ||
            dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE_)
            need_move = true;
    }
    if (!need_move) return;

    /*need_move true니까 이동준비*/
    BoxPointType New_LocalMap_Points = local_map_points_;
    BoxPointType tmp_boxpoints;

    float mov_dist = max((params_.box_len - 2.0 * MOV_THRESHOLD * DET_RANGE_) * 0.5 * 0.9,
                        double(DET_RANGE_ * (MOV_THRESHOLD - 1)));
    //한번에 local map box 이동시킬지 계산

    /*각 축별로 box이동 */
    for (int i = 0; i < 3; i++)
    {
        tmp_boxpoints = local_map_points_;
        /*min쪽에 가까움*/
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE_)
        {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = local_map_points_.vertex_max[i] - mov_dist;
            cub_need_rm_.push_back(tmp_boxpoints);
            //기존 local_map_points가지고 있던 것은 삭제 목록으로 이동
        }
        /*max에 가까움*/
        else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE_)
        {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = local_map_points_.vertex_min[i] + mov_dist;
            cub_need_rm_.push_back(tmp_boxpoints);
        }
    }
    local_map_points_ = New_LocalMap_Points;
    //local map 갱신


    
    {
        /*ikd-tree에 쌓여 있던 이전에 삭제된 point 기록을 꺼내서 비움*/
        PointVector points_history;
        map_.acquireRemovedPoints(points_history);
        //ikd-tree 내부 버퍼를 비우기 위움
        //lib 내부적으로 Point_deleted버퍼에 쌓아두는데,  주기적으로 버퍼를 flush한다.
        //단지 값은 안넣고, 삭제 함수를 사용하기 위한 용도다. 

    }

    /*삭제 후보 box들을 ikd-tree에서 삭제*/
    double delete_begin = omp_get_wtime();
    if (!cub_need_rm_.empty())
        kdtree_delete_counter_ = map_.deleteBoxes(cub_need_rm_);
        //cub_need_rm_안에 들어가 있는 box영역에 포함된 map point들을 삭제함
    kdtree_delete_time_ = omp_get_wtime() - delete_begin;
}

void SlamCore::mapIncremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size_);
    PointNoNeedDownsample.reserve(feats_down_size_);

    for (int i = 0; i < feats_down_size_; i++)
    {
        pointBodyToWorld(&(feats_down_body_->points[i]), &(feats_down_world_->points[i]));

        if (!nearest_points_[i].empty() && flg_EKF_inited_)
        {
            const PointVector& points_near = nearest_points_[i];
            bool need_add = true;
            PointType mid_point;
            mid_point.x = floor(feats_down_world_->points[i].x / filter_size_map_min_) * filter_size_map_min_ + 0.5f * filter_size_map_min_;
            mid_point.y = floor(feats_down_world_->points[i].y / filter_size_map_min_) * filter_size_map_min_ + 0.5f * filter_size_map_min_;
            mid_point.z = floor(feats_down_world_->points[i].z / filter_size_map_min_) * filter_size_map_min_ + 0.5f * filter_size_map_min_;
            float dist = calc_dist(feats_down_world_->points[i], mid_point);

            if (fabs(points_near[0].x - mid_point.x) > 0.5f * filter_size_map_min_ &&
                fabs(points_near[0].y - mid_point.y) > 0.5f * filter_size_map_min_ &&
                fabs(points_near[0].z - mid_point.z) > 0.5f * filter_size_map_min_)
            {
                PointNoNeedDownsample.push_back(feats_down_world_->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++)
            {
                if ((int)points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist) { need_add = false; break; }
            }
            if (need_add) PointToAdd.push_back(feats_down_world_->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world_->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size_ = map_.addPoints(PointToAdd, true);
    map_.addPoints(PointNoNeedDownsample, false);
    add_point_size_ = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time_ = omp_get_wtime() - st_time;
}

void SlamCore::accumulatePublishCloud()
{
    int sz = feats_undistort_->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(sz, 1));
    for (int i = 0; i < sz; i++)
        rgbPointBodyToWorld(&feats_undistort_->points[i], &laserCloudWorld->points[i]);
    *pcl_wait_pub_ += *laserCloudWorld;
}

// ─────────────────────────────────────────────────────────────────────────────
// Layer 7: Output / Debug
// ─────────────────────────────────────────────────────────────────────────────
void SlamCore::updateSnapshot(bool valid)
{
    SlamSnapshot snap;
    snap.valid          = valid;
    snap.lidar_end_time = lidar_end_time_;

    if (valid)
    {
        snap.pose.pos          = state_point_.pos;
        snap.pose.rot          = state_point_.rot;
        snap.pose.vel          = state_point_.vel;
        snap.pose.offset_R_L_I = state_point_.offset_R_L_I.toRotationMatrix();
        snap.pose.offset_T_L_I = state_point_.offset_T_L_I;

        snap.geoQuat.x = state_point_.rot.coeffs()[0];
        snap.geoQuat.y = state_point_.rot.coeffs()[1];
        snap.geoQuat.z = state_point_.rot.coeffs()[2];
        snap.geoQuat.w = state_point_.rot.coeffs()[3];

        snap.cov_P           = iekf_.get_P();
        snap.feats_undistort = feats_undistort_;
        snap.feats_down_body = feats_down_body_;
        snap.laserCloudOri   = laserCloudOri_;
        snap.pcl_wait_pub    = pcl_wait_pub_;
        snap.effct_feat_num  = effct_feat_num_;
    }

    std::lock_guard<std::mutex> lk(snapshot_mutex_);
    latest_snapshot_ = snap;
}

void SlamCore::dumpLioStateToLog()
{
    if (!fp_) return;
    V3D rot_ang(Log(state_point_.rot.toRotationMatrix()));
    fprintf(fp_, "%lf ", Measures_.lidar_beg_time - first_lidar_time_);
    fprintf(fp_, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));
    fprintf(fp_, "%lf %lf %lf ", state_point_.pos(0), state_point_.pos(1), state_point_.pos(2));
    fprintf(fp_, "%lf %lf %lf ", 0.0, 0.0, 0.0);
    fprintf(fp_, "%lf %lf %lf ", state_point_.vel(0), state_point_.vel(1), state_point_.vel(2));
    fprintf(fp_, "%lf %lf %lf ", 0.0, 0.0, 0.0);
    fprintf(fp_, "%lf %lf %lf ", state_point_.bg(0), state_point_.bg(1), state_point_.bg(2));
    fprintf(fp_, "%lf %lf %lf ", state_point_.ba(0), state_point_.ba(1), state_point_.ba(2));
    fprintf(fp_, "%lf %lf %lf ", state_point_.grav[0], state_point_.grav[1], state_point_.grav[2]);
    fprintf(fp_, "\r\n");
    fflush(fp_);
}

void SlamCore::dumpPerformanceLog(double t0, double t3,
                                   double t_update_start, double t_update_end,
                                   double solve_H_time)
{
    frame_num_++;
    aver_time_consu_        = aver_time_consu_        * (frame_num_-1)/frame_num_ + (t3-t0)/frame_num_;
    aver_time_icp_          = aver_time_icp_          * (frame_num_-1)/frame_num_ + (t_update_end-t_update_start)/frame_num_;
    aver_time_match_        = aver_time_match_        * (frame_num_-1)/frame_num_ + match_time_/frame_num_;
    aver_time_incre_        = aver_time_incre_        * (frame_num_-1)/frame_num_ + kdtree_incremental_time_/frame_num_;
    aver_time_solve_        = aver_time_solve_        * (frame_num_-1)/frame_num_ + solve_time_/frame_num_;
    aver_time_const_H_time_ = aver_time_const_H_time_ * (frame_num_-1)/frame_num_ + solve_H_time/frame_num_;

    V3D ext_euler = SO3ToEuler(state_point_.offset_R_L_I);
    fout_out_ << std::setw(20) << Measures_.lidar_beg_time - first_lidar_time_
              << " " << euler_cur_.transpose()
              << " " << state_point_.pos.transpose()
              << " " << ext_euler.transpose()
              << " " << state_point_.offset_T_L_I.transpose()
              << " " << state_point_.vel.transpose()
              << " " << state_point_.bg.transpose()
              << " " << state_point_.ba.transpose()
              << " " << state_point_.grav
              << " " << feats_undistort_->points.size() << "\n";

    dumpLioStateToLog();
}

// ─────────────────────────────────────────────────────────────────────────────
// 공통 헬퍼
// ─────────────────────────────────────────────────────────────────────────────
bool SlamCore::syncPackages(MeasureGroup& meas)
{
    if (lidar_buffer_.empty() || imu_buffer_.empty()) return false;

    if (!lidar_pushed_)
    {
        meas.lidar          = lidar_buffer_.front();
        meas.lidar_beg_time = time_buffer_.front();

        if (meas.lidar->points.size() <= 1)
        {
            lidar_end_time_ = meas.lidar_beg_time + lidar_mean_scantime_;
            std::cerr << "Too few input point cloud!\n";
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime_)
        {
            lidar_end_time_ = meas.lidar_beg_time + lidar_mean_scantime_;
        }
        else
        {
            scan_num_++;
            lidar_end_time_ = meas.lidar_beg_time +
                              meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime_ +=
                (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime_) / scan_num_;
        }
        meas.lidar_end_time = lidar_end_time_;
        lidar_pushed_       = true;
    }

    if (last_timestamp_imu_ < lidar_end_time_) return false;

    double imu_time = imu_buffer_.front().timestamp;
    meas.imu.clear();
    while (!imu_buffer_.empty() && imu_time < lidar_end_time_)
    {
        imu_time = imu_buffer_.front().timestamp;
        if (imu_time > lidar_end_time_) break;
        meas.imu.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
    }
    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    lidar_pushed_ = false;
    return true;
}

void SlamCore::pointBodyToWorld(PointType const* pi, PointType* po) const
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point_.rot * (state_point_.offset_R_L_I * p_body + state_point_.offset_T_L_I)
                 + state_point_.pos);
    po->x = p_global(0); po->y = p_global(1); po->z = p_global(2);
    po->intensity = pi->intensity;
}

void SlamCore::rgbPointBodyToWorld(PointType const* pi, PointType* po) const
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point_.rot * (state_point_.offset_R_L_I * p_body + state_point_.offset_T_L_I)
                 + state_point_.pos);
    po->x = p_global(0); po->y = p_global(1); po->z = p_global(2);
    po->intensity = pi->intensity;
}
