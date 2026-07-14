#pragma once

#include <Eigen/Dense>
#include "third_party/IKFoM_toolkit/esekfom/esekfom.hpp"

#include "core/api/use_ikfom.hpp"


#include "core/types/slam_types.hpp"
#include "core/api/ikd_tree_api.hpp"

// #include "core/types/common.hpp"



// #include <Eigen/Core>
/*
Esekfom : Error-State Extended Kalman Filter on Manifold
        이건 일반 벡터가아니라, pose, rot, grav와 같은 상태를 다루는 Error-State EKF 엔진이다.


EsekfomApi는 third party EKF엔진을 쓰기 위한 wrapping 클래스다. 
IKFoM : Iterated Kalman Filter on Manifolds
esekfom : Error-State EKF on Manifold 계열의 필터 엔진 namespace/class
    여기에 Manifold라는 형식을 쓰기 때문에 이에 맞는 EKF엔진으로 고쳐야 한다. 

담당할것
1. iEKF state 보관
2. covariance 보관
3. IMU predict 호출
4. LiDAR update 호출
5. 현재 pose /velocity /bais 꺼내기
6. 초기 state 설정

목표
ImuProcessor : IMU 데이터 준비 담당
-> dt, acc_avg, gyr_avg 계산

EsekfomApi : EKF state 실제 갱신 담당
->iput_ikfom 생성
->kf_.predict(dt,Q,input) 호출
->state_ikfom 갱신



*/
struct LidarUpdateContext
{
    CloudTPtr cloud_body_down;
    const IkdTreeApi* ikd_tree = nullptr;

    int nearest_num = 5;
    double dist5_thresh = 1.0;
    double residual_thresh = 0.2;
};

class EsekfomApi
{
public:
    EsekfomApi();
    ~EsekfomApi() = default;

    void setParams(const SlamParams& params);
    //yaml에서 noise , extrinsic, iteration 값 저장
    
    void initFromImuMean(const V3D& mean_acc, const V3D& mean_gyr);
    //정지상태의 IMU평균값으로 초기 state를 설정,

    void predictImu(double dt, const V3D& acc, const V3D& gyr, const Q12& Q);
    /*
        IMU 한 구간 dt에 대해 iEKF predict를 수행한다.

        내부 흐름: 
        acc/gyr 
        -> input_ikfom 생성
        -> kf_.predict(dt, Q, input)
        -> state_ikfom 예측
    */

    void updateResidual(double lidar_point_cov, double& solve_time);
    //나중에 LiDAR residual 기반 update에서 사용함
    
    /*디버그 / 후속 처리용 state getter*/
    Eigen::Vector3d getPosition() const;
    Eigen::Vector3d getVelocity() const;
    Eigen::Matrix3d getRotationMatrix() const;
    state_ikfom getState() const ;


    PoseState getPoseState() const;

    // void propagateOnce();
    // void updateOnce();
    void applyPoseCorrection(const Eigen::Matrix<double, 6, 1>& dx);

    bool updateLidarWithMap(const CloudTPtr& cloud_body_down, const IkdTreeApi& ikd_tree, double lidar_point_cov, double& solve_time);

public: //params

private: 

private:  //parmas
    SlamParams params_;
    //IEkf iekf_;
    esekfom::esekf<state_ikfom, 12,input_ikfom> kf_;
    // third_party IKFoM/esskfom 실제 필터 엔진
    // state_ikfom : FAST_LIO2용 state
    // 12  : precess noise dimension
    // input_ikfom : IMU input


    
    bool is_filter_initialized_ = false;
    bool has_params_ = false;
    int imu_count_ =0;

    LidarUpdateContext lidar_update_context_;
    static EsekfomApi* active_instance_;

    static void hShareModelWrapper(
        state_ikfom& s,
        esekfom::dyn_share_datastruct<double>& ekfom_data);

    void hShareModel(
        state_ikfom& s,
        esekfom::dyn_share_datastruct<double>& ekfom_data);

    int last_lidar_effective_num_ = 0;
    bool last_lidar_update_valid_ = false;
};