#pragma once
#include <mutex>
#include <deque>
#include <vector>
#include <string>
#include <fstream>
#include <memory>

#include <common_lib.h>
#include <use-ikfom.hpp>
#include <third_party/IKFoM_toolkit/esekfom/esekfom.hpp>
#include <third_party/ikd-Tree/ikd_Tree.h>
#include <pcl/filters/voxel_grid.h>

#include "core/slam_types.hpp"
#include "algorithm/iekf.hpp"
#include "algorithm/map_manager.hpp"
#include "algorithm/imu_processor.hpp"


/*
전체 순서
LiDAR  scan input

→ IMU deskew

→ downsmaple 

→ 현재 pose 기준 world 변환

→ local map 범위 갱신

→ikd-tree에서 nearest search

→ plane residual /Jacobian 생성

→ IKFoM iEKF update

→ corrected pose 기준 map에 point추가
*/

class SlamCore
{
public:
    SlamCore();
    ~SlamCore();

    void init(const SlamParams& params);
    void pushLidar(std::shared_ptr<PointCloudXYZI> cloud, double timestamp);
    void pushImu(const ImuData& imu);
    void spinFrontendOnce();
    SlamSnapshot getSnapshot() const;
    bool saveMap() const;

private:
    // ── 알고리즘 컴포넌트 ─────────────────────────────────────────────────────
    ImuProcess p_imu_;   // IMU 초기화 + predict + undistortion
    IEkf       iekf_;    // iEKF 상태 추정기
    MapManager map_;     // IKD-Tree 맵

    // ── 현재 추정 상태 ────────────────────────────────────────────────────────
    state_ikfom  state_point_;   // EKF 상태 (pos, rot, vel, bias, gravity, extrinsic)
    vect3        pos_lid_;       // LiDAR 위치 (world frame)
    V3D          euler_cur_;     // 오일러각 (디버그용)
    MeasureGroup Measures_;      // 현재 프레임의 LiDAR+IMU 묶음

    // ── 입력 버퍼 ─────────────────────────────────────────────────────────────
    std::mutex   mtx_buffer_;
    std::deque<double>                                time_buffer_;
    std::deque<PointCloudXYZI::Ptr>                   lidar_buffer_;
    std::deque<ImuData>                               imu_buffer_;

    // ── 포인트 클라우드 작업 버퍼 ─────────────────────────────────────────────
    PointCloudXYZI::Ptr featsFromMap_;
    PointCloudXYZI::Ptr feats_undistort_;   // undistortion 결과
    PointCloudXYZI::Ptr feats_down_body_;   // body frame 다운샘플
    PointCloudXYZI::Ptr feats_down_world_;  // world frame 다운샘플
    PointCloudXYZI::Ptr normvec_;           // 평면 법선벡터
    PointCloudXYZI::Ptr laserCloudOri_;     // effective 포인트 (body)
    PointCloudXYZI::Ptr corr_normvect_;     // effective 포인트의 법선
    PointCloudXYZI::Ptr _featsArray_;
    PointCloudXYZI::Ptr pcl_wait_pub_;      // 누적 맵 (publish용)

    // ── 맵 FOV 관리 ───────────────────────────────────────────────────────────
    /*지금 ikd-tree가 관리하는 local map의 공간 범위*/
    //FOV : Field Of View (관심영역)
    BoxPointType              local_map_points_; // 현재 로봇 주변 local map 3d박스 범위
    //전체 무한 map을 다루지 않고, 일정영역만 local map으로 유지. 그래야 nearest search빠름

    bool                      localmap_initialized_ = false; //한번이라도 box가 초기화 되었는지 
    //처음으로 pose가 잡히고 LiDAR 위치가 생기면, 로봇 위치를 중심으로 local map box 만듬.

    std::vector<BoxPointType>       cub_need_rm_;             // FOV 벗어나서 삭제할 박스들

    /*scan-to-map matching용*/
    std::vector<PointVector>        nearest_points_;         // 각 피처의 nearest 이웃
    //query point와 가까운 5개점으로 후에 plane 추정함
    //i번째 query point 주변에서 찾은 nearest map point 자체

    // std::vector<std::vector<int>>   pointSearchInd_surf_;    // 서프 피처 인덱스
    //i번째 query point 주변에서 찾은 nearest map point들의 index 목록
    //현재 안씀

    // ── 다운샘플 필터 ─────────────────────────────────────────────────────────
    pcl::VoxelGrid<PointType> downSizeFilterSurf_;
    pcl::VoxelGrid<PointType> downSizeFilterMap_;

    // ── 타이밍 & 카운터 ───────────────────────────────────────────────────────
    double last_timestamp_lidar_     = 0.0;
    double last_timestamp_imu_       = -1.0;
    bool   lidar_pushed_             = false;
    bool   is_first_lidar_           = true;
    double timediff_lidar_wrt_imu_   = 0.0;
    bool   timediff_set_flg_         = false;
    double lidar_mean_scantime_      = 0.0;
    int    scan_num_                 = 0;
    double lidar_end_time_           = 0.0;
    double first_lidar_time_         = 0.0;
    bool   flg_first_scan_           = true;
    bool   flg_EKF_inited_           = false;

    // ── 피처 포인트 상태 ──────────────────────────────────────────────────────
    float  res_last_[100000];          // 포인트별 잔차
    bool   point_selected_surf_[100000]; // 유효 피처 마스크
    int    feats_down_size_    = 0;
    int    effct_feat_num_     = 0;
    double res_mean_last_      = 0.05;
    double total_residual_     = 0.0;

    // ── 파라미터 ──────────────────────────────────────────────────────────────
    SlamParams  params_;
    float       DET_RANGE_              = 300.0f;
    
    double      filter_size_map_min_    = 0.5;
    double      filter_size_surf_min_   = 0.5;
    bool        extrinsic_est_en_       = true;
    bool        time_sync_en_           = false;
    double      time_diff_lidar_to_imu_ = 0.0;
    bool        runtime_pos_log_        = false;
    bool        pcd_save_en_            = false;
    int         NUM_MAX_ITERATIONS_     = 4;
    std::string map_file_path_;
    std::string root_dir_;

    // ── 보조 기하 ─────────────────────────────────────────────────────────────
    V3D position_last_   = Zero3d;
    V3D Lidar_T_wrt_IMU_ = Zero3d;
    M3D Lidar_R_wrt_IMU_ = Eye3d;
    V3F XAxisPoint_body_;
    V3F XAxisPoint_world_;

    // ── 프로파일링 ────────────────────────────────────────────────────────────
    double match_time_              = 0.0;
    double kdtree_search_time_      = 0.0;
    double solve_time_              = 0.0;
    double kdtree_incremental_time_ = 0.0;
    double kdtree_delete_time_      = 0.0;
    int    kdtree_size_st_          = 0;
    int    kdtree_size_end_         = 0;
    int    add_point_size_          = 0;
    int    kdtree_delete_counter_   = 0;
    int    scan_count_              = 0;
    int    publish_count_           = 0;
    double aver_time_consu_         = 0;
    double aver_time_icp_           = 0;
    double aver_time_match_         = 0;
    double aver_time_incre_         = 0;
    double aver_time_solve_         = 0;
    double aver_time_const_H_time_  = 0;
    int    frame_num_               = 0;

    // ── 로그 파일 ─────────────────────────────────────────────────────────────
    FILE*         fp_      = nullptr;
    std::ofstream fout_pre_, fout_out_;

    // ── 스냅샷 (RosBridge 에 넘길 결과) ─────────────────────────────────────
    mutable std::mutex snapshot_mutex_;
    SlamSnapshot       latest_snapshot_;

    // ─────────────────────────────────────────────────────────────────────────
    // Layer 2 : Preprocess / Deskew
    // ─────────────────────────────────────────────────────────────────────────
    void runDeskew();           // IMU propagation → predicted state + undistorted cloud
    void downsampleFeatures();  // VoxelGrid : feats_undistort → feats_down_body

    // ─────────────────────────────────────────────────────────────────────────
    // Layer 4 : Measurement Model
    //   esekfom 이 raw function pointer 를 요구 → singleton + static wrapper
    // ─────────────────────────────────────────────────────────────────────────
    static SlamCore* s_instance_;
    static void h_share_model_static(state_ikfom& s,
                                     esekfom::dyn_share_datastruct<double>& ekfom_data);
    void h_share_model(state_ikfom& s,
                       esekfom::dyn_share_datastruct<double>& ekfom_data);
    // nearest search → plane fitting → residual → Jacobian

    // ─────────────────────────────────────────────────────────────────────────
    // Layer 5 : Filter Engine
    // ─────────────────────────────────────────────────────────────────────────
    void runIEKFUpdate(double& solve_H_time);
    // iEKF iterated update (내부에서 h_share_model 반복 호출) → corrected state

    // ─────────────────────────────────────────────────────────────────────────
    // Layer 6 : Map Manager
    // ─────────────────────────────────────────────────────────────────────────
    void buildInitialMap();     // 첫 프레임 : ikd-tree 초기 빌드
    void lasermapFovSegment();  // 슬라이딩 윈도우 : 범위 밖 포인트 삭제
    void mapIncremental();      // 보정된 포즈로 새 포인트 ikd-tree 에 추가
    void accumulatePublishCloud(); // publish 용 누적 맵 클라우드 갱신

    // ─────────────────────────────────────────────────────────────────────────
    // Layer 7 : Output / Debug
    // ─────────────────────────────────────────────────────────────────────────
    void updateSnapshot(bool valid);
    void dumpLioStateToLog();
    void dumpPerformanceLog(double t0, double t3,
                            double t_update_start, double t_update_end,
                            double solve_H_time);

    // ─────────────────────────────────────────────────────────────────────────
    // 공통 헬퍼
    // ─────────────────────────────────────────────────────────────────────────
    bool syncPackages(MeasureGroup& meas);
    void pointBodyToWorld(PointType const* pi, PointType* po) const;
    void rgbPointBodyToWorld(PointType const* pi, PointType* po) const;
};
