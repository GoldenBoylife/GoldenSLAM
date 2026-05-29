#pragma once
#include <deque>
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>

#include <common_lib.h>
#include "algorithm/iekf.hpp"

// IMU 초기화 + 상태 예측 + LiDAR 포인트 deskewing
// MeasureGroup (LiDAR 1프레임 + 그 사이 IMU들) 을 받아서:
//   1. 초기 프레임: 중력/자이로 편향 초기화
//   2. 이후: IEkf.predict() 로 IMU 적분 → 포인트 undistortion
class ImuProcess
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuProcess();
    ~ImuProcess();

    void Reset();
    void Reset(double start_timestamp, const ImuData& lastimu);

    void set_extrinsic(const V3D& transl, const M3D& rot);
    void set_extrinsic(const V3D& transl);
    void set_extrinsic(const MD(4,4)& T);
    void set_gyr_cov(const V3D& scaler);
    void set_acc_cov(const V3D& scaler);
    void set_gyr_bias_cov(const V3D& b_g);
    void set_acc_bias_cov(const V3D& b_a);

    // 메인 처리: IEkf 를 직접 참조해서 predict/init 수행
    void Process(const MeasureGroup& meas, IEkf& kf, PointCloudXYZI::Ptr pcl_un_);

    Eigen::Matrix<double, 12, 12> Q;

    std::ofstream fout_imu;
    V3D cov_acc;
    V3D cov_gyr;
    V3D cov_acc_scale;
    V3D cov_gyr_scale;
    V3D cov_bias_gyr;
    V3D cov_bias_acc;
    double first_lidar_time;

private:
    void IMU_init(const MeasureGroup& meas, IEkf& kf, int& N);
    void UndistortPcl(const MeasureGroup& meas, IEkf& kf, PointCloudXYZI& pcl_out);

    PointCloudXYZI::Ptr cur_pcl_un_;
    ImuData               last_imu_;
    std::deque<ImuData>   v_imu_;
    std::vector<Pose6D> IMUpose;
    std::vector<M3D>    v_rot_pcl_;
    M3D    Lidar_R_wrt_IMU;
    V3D    Lidar_T_wrt_IMU;
    V3D    mean_acc;
    V3D    mean_gyr;
    V3D    angvel_last;
    V3D    acc_s_last;
    double start_timestamp_;
    double last_lidar_end_time_;
    int    init_iter_num  = 1;
    bool   b_first_frame_ = true;
    bool   imu_need_init_ = true;
};
