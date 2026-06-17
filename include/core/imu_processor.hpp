#pragma once
#include <deque>
#include <vector>
#include <cmath>
#include <iostream>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>


#include "core/types/slam_types.hpp"
#include "api/esekfom_api.hpp" //이걸써?


struct ImuPoseHistory
{
    double  offset_time = 0.0; //LiDAR frame begin 기준 상대시간[sec]
    V3D pos = V3D::Zero();
    M3D rot = M3D::Identity();  
};

/*
    IMU 초기화 + 상태 예측 + LiDAR 포인트 deskew

    MeasureGroup 을 받아서, 
    1. 초기 프레임 : 중력 /자이로 편향 초기화
    2. IEkf.predict()로 IMU 적분하여 IMU propagation
    3. LiDAR point undistortion

*/
class ImuProcessor 
{
public: 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuProcessor();
    ~ImuProcessor();

    void reset();
    void setFirstLidarTime(double first_lidar_time);

    // void reset(double start_timestamp, const ImuData& last_imu);
    
    void set_extrinsic(const V3D& transl, const M3D& rot);
    void set_gyr_cov(const V3D& scaler);
    void set_acc_cov(const V3D& scaler);
    void set_gyr_bias_cov(const V3D& b_g);
    void set_acc_bias_cov(const V3D& b_a);


    void process(const MeasureGroup& meas, EsekfomApi& eskfom_api);
    /*메인처리 : IEkf를 직접 참조해서 predict/init 수행*/
    // void process(const MeasureGroup& meas, IEkf& kf)
    //이거 runDeskew할때 해결해야 함. 

    void savePoseHistory(double offset_time, const EsekfomApi& esekfom_api) ;
        //IMU propagation 이후의 state를 pose_history에 저장

    void debugPointTimeAndPoseHistory(const CloudT::Ptr& cloud, double lidar_beg_time, double lidar_end_time) const;


public: //(param)



private:
    void imuInit(const MeasureGroup& meas, EsekfomApi& esekfom_api);
    //초기 IMU 평균으로 gravity, gyro bias, covariance 초기화

    void propagateImu(const MeasureGroup& meas, EsekfomApi& esekfom_api);
    //MeasureGroup 안의 IMU들을 순회하면서 EsekfomApi::predictImu()호출

    void undistort(const MeasureGroup& meas, EsekfomApi& esekfom_api);
    //pose history를 사용해서 LiDAR point deskew


private: //(param)

    static constexpr int MAX_INIT_COUNT =10;
    static constexpr double G_M_S2 = 9.81;

    // 260604새벽1시 지금. 여기까지 했음. imu_processor만들고 ,cpp만들어서, 
    // propagation까지하고 디버깅 하려던 참임

    bool b_first_frame_;
    bool imu_need_init_;

    int init_iter_num_;
    
    double  first_lidar_time_;
    double last_lidar_end_time_;

    ImuData last_imu_;

    V3D mean_acc_;
    V3D mean_gyr_;

    V3D cov_acc_; //가속도계 x,y,z noise 크기
    V3D cov_gyr_; //자이로 x,y,z noise 크기
    
    V3D cov_bias_gyr_;
    V3D cov_bias_acc_;

    V3D cov_acc_scale_;
    V3D cov_gyr_scale_;


    Eigen::Matrix<double, 12, 12> Q_;
    //이거  undistort할때 씀
    //Q와 P는 둘다 covariance로 분산값이지만 대상이 다르다. 
    //P : 이번 state 자체가 얼마나 불확실한가.
    //Q : 이번 IMU prediction 과정에서 새로 추가되는 noise가 얼마나 큰가?
    //gyro noise, acc noise, gyro bias random walk, acc bias random walk,

    std::vector<ImuPoseHistory> imu_pose_history_;
    


};