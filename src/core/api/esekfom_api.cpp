#include "include/core/api/esekfom_api.hpp"
#include <iostream>

namespace 
{
    /*
    아직 LiDAR update용 h_share_model을 연결하지 않았으므로 일시 stub를 둔다.

    현재 브랜치의 목적은 IMU propagation 연결이다. 
    실제 LiDAR residual update는 나중에 별도 단계에서 구현한다.
    */
   void hShareModelStub( state_ikfom& s, esekfom::dyn_share_datastruct<double>& esekfom_data)
   {
        (void)s;

        /*
            LiDAR update를 아직 하지 않겠다는 뜻.
            esekfom 버전에 따라 valid 필드가 없으면 이중에서 컴파일 에러가 날 수 있다. 
            그 경우 이 함수 안을 완전히 비워도 된다.
        */
       esekfom_data.valid = false;
    
   }
}


EsekfomApi::EsekfomApi()
{
}

void EsekfomApi::setParams(const SlamParams& params)
{
    std::cout << "[EsekfomApi::setParams] this=" << this
              << " before initialized=" << is_filter_initialized_
              << std::endl;

    params_ = params;
    has_params_ = true;

    /*
        Q는 ImuProcessor가 보유한다. 따라서 EsekfomApi에서는 Q_안ㅆ느다.
    */
//    Q_ = process_noise_cov();

   /*
        esekfom 초기화

        get_f : state 동역학 모델
        df_dx : state Jacobian
        df_dw : noise Jacobian
        hShareModelStub : 아직 LiDAR updqt가 없으므로 stub 사용
   */

   constexpr int NUM_MAX_ITERATIONS =4;
//    std::vector<double> epsi(23, 0.001);
    double epsi[23];
    for(int i =0; i < 23; ++i) 
    {
        epsi[i] = 0.001;
    }

   kf_.init_dyn_share(
    get_f,
    df_dx,
    df_dw,
    hShareModelStub,
    NUM_MAX_ITERATIONS,
    epsi
   );


   is_filter_initialized_ = true; 
    std::cout << "[EsekfomApi::setParams] this=" << this
              << " after initialized=" << is_filter_initialized_
              << std::endl;
    
}

/*
imu 초기화가 끝났을때 state를 셋팅
    imuInit() 초기 IMU 평균값을 계산하는 함수라면, 
    여기서는 그 평균값을 가지고 EKF필터의 초기 state를 실제로 세팅함.
    그 평균값으로 kf_내부에서 state_ikfom /covariance P를 초기화


*/
void EsekfomApi::initFromImuMean(const V3D& mean_acc, const V3D& mean_gyr) 
{
    if(!is_filter_initialized_) 
    {
        std::cerr << " filter is not initialized." 
        << "call setParams() first." << std::endl;
        return;
    }

    if(mean_acc.norm() < 1e-6)
    {
        std::cerr << "[EsekfomApi::initFromImuMean] invalid mean_acc norm."
        << std::endl;
        return; 
    }

    state_ikfom init_state = kf_.get_x();

    //x :  현재 EKF 필터 안에 들어 있는 state를 꺼냄.

    /*
        초기위치/속도 / acc bias는 명시적으로 0으로 둔다.
    */
    init_state.pos = V3D::Zero();
    init_state.vel = V3D::Zero();
    init_state.ba = V3D::Zero();
    /*
    초기 gyro bias설정.
    정지 상태에서 측정된 gyro 평균은
    실제 회전값이 아니라 gyro bias로 본다.
    */
   for(int i =0; i < 3 ;++i)
   {
       init_state.bg[i] = mean_gyr[i];
   }

   /* 
   초기 gravity 설정. 
   정지 상태에서 acceleromter 평균 방향은 중력 방향을 알려준다.

   mean_acc는 IMU가 측정한 중력 방향이고, 
   state의 grav는 world frame gravity로 들어간다. 
   근데 왜 여기서 R이 안붙냐면, 현재 시작 단계이기 때문에 굳이 
   R이 필요 없는 것이다. 그래서 생략 된다.
   */
  constexpr double G_M_S2 = 9.81;
  Eigen::Vector3d grav_vec = -mean_acc.normalized() * G_M_S2;
  // vel_dot = R * (acc-ba) + grav에서, 
  // 정지상태면, vel_dot 이 0에 가까워야한다.
  // R* (acc-ba) + grav = 0이니까.  grav = -R*(acc-ba)된다.
   //mean_acc.normalized() : 방향만 남긴다. 
   // 예를 들어서, mean_acc = [0.2, -0.1, 9.7] 라면,  방향 단위벡터로 만들어서 길이 1이 되도록 만든다.
   // 그 뒤에 9.81을 곱한다.
   // 여기서 (acc-ba)이어야 하는데 ba가 초기니까 0으로 본듯.
  init_state.grav = S2(grav_vec);
   //world frame기준 state, 


  /*
    extrinsic 은 일단 기본값 유지. 
    나중에 params_에서 offset_R_L_I, offset_T_L_I를 읽어와서 넣으면 된다.

  */
   kf_.change_x(init_state);
   //위에서 만든 state들 안의 grav와 bg를 여기에 넣고  초기값으로 바꿔줌.

    /*
        진짜 EKF 내부 state에 bg, grav가 들어갔는지 확인
    */
    {
        const state_ikfom check_state = kf_.get_x();

        std::cout << "[EsekfomApi::initFromImuMean][check state] "
                << " bg=" << check_state.bg.transpose()
                << " ba=" << check_state.ba.transpose()
                << " grav=" << grav_vec.transpose()
                << std::endl;
    }
   /*
    각 state 항목을 얼마나 믿을지 설정하기. 

    초기 cov P 설정
    cov는 분산/공분산임. 
    이때 EKF에서 P는  어떤 센서의 분산 하나값이 아니라, 
    현재 EKF state 전체에 대한 불확실성 행렬이다.

    state_ikfom의 error state차원은 23이다. 그래서 보통 Psms 23x23 matrix다.

    0~2 : pose err
    3~5 : rot err
    6~8 : offset_R_L_I err
    9~11 : offset_T_L_I err
    12~14 : vel err
    15~17 : gyro bias err
    18~20 : acc bias err
    21~22 : gravity err

    대각선 값은 state err 의 분산값. 
   */
  auto P = kf_.get_P();
  P.setIdentity();
  P(6,6) = 0.00001; 
  P(7,7) = 0.00001; 
  P(8,8) = 0.00001;  

  
  P(9,9) = 0.00001;
  P(10,10) = 0.00001;
  P(11,11) = 0.00001;
  

  P(15,15) = 0.0001;
  P(16,16) = 0.0001;
  P(17,17) = 0.0001;

  P(18,18) = 0.001;
  P(19,19) = 0.001;
  P(20,20) = 0.001;

  P(21,21) = 0.00001;
  P(22,22) = 0.00001;

  kf_.change_P(P);
  // 분산 
  


  /*
   너무 큰 uncertainty로 시작하지않도록 일부 항목을 작게 둔다.
   정확한 값은 나중에 YAML 기준으로 조정한다.
  */

std::cout << "[EsekfomApi::initFromImuMean] initialized "
              << " mean_acc=" << mean_acc.transpose()
              << " mean_gyr=" << mean_gyr.transpose()
              << " grav=" << grav_vec.transpose()
              << std::endl;
}
/*
imu propagation 실제입구

ImuProcessor 가 만든 dt, acc_avg gyr_avg, Q를 esekfom 형식으로 바꿔서 kf_.predict()를 호출함



IMU raw값에 각속도와 가속도를 있다.
따라서 dt를 알아내고, 이걸로 pose와 velocity를 유추하는 거다.

이전에 propagateImu에서
    - dt, acc 평균, gyro 평균 계산했다.
여기서는 
    - acc_avg, gyr_avg를 받아서 input_ikfom으로 변환
    - iekf_.predict()

*/
void EsekfomApi::predictImu(double dt, const V3D& acc, const V3D& gyr, const Q12& Q)
{
    if (!is_filter_initialized_)
    {
        std::cerr << "[EsekfomApi::predictImu] filter is not initialized. "
                  << "Call setParams() first." << std::endl;
        return;
    }


    if(dt <= 0.0)   return;


    /*
    Eigen Vector3d 형태의 acc/gyr를 IKFoM용 input_ikfom 으로 변환한다.
    */

    input_ikfom in;
    for(int i =0; i < 3 ; ++i) 
    {
        in.acc[i] = acc[i];
        in.gyro[i] = gyr[i];
    }


    /*
        kf_.predict()가 Q를 non-const reference를 받을 수 있으므로, cosnt Q를 그대로 넘기지 않고 복사본을 만든다.
    */
   Q12 Q_copy = Q;

   /*
        실제 IMU propagation.
        내부적으로 use-ikfom.hpp의 get_f(), df_dx(), df_dw()를 사용해서 state_ikfom과 cov P를 앞으로 예측한다.
   */

static int debug_count = 0;

if (debug_count < 20)
{
    const state_ikfom s = kf_.get_x();

    const V3D gyr_corr = gyr - s.bg;
    const V3D acc_corr = acc - s.ba;

    std::cout << "[EsekfomApi::predictImu][debug] "
              << " dt=" << dt
              << " gyr_raw=" << gyr.transpose()
              << " bg=" << s.bg.transpose()
              << " gyr_corr=" << gyr_corr.transpose()
              << " acc_raw=" << acc.transpose()
              << " ba=" << s.ba.transpose()
              << " acc_corr=" << acc_corr.transpose()
              << std::endl;

    ++debug_count;
}


   kf_.predict(dt, Q_copy, in);

   ++imu_count_;

}




/*Getter들 */

Eigen::Vector3d EsekfomApi::getPosition() const 
{
    const state_ikfom state = kf_.get_x();
    return Eigen::Vector3d(
        state.pos[0],
        state.pos[1],
        state.pos[2]);
    
}


Eigen::Vector3d EsekfomApi::getVelocity() const
{
    const state_ikfom state = kf_.get_x();  
    return Eigen::Vector3d(
        state.vel[0],
        state.vel[1],
        state.vel[2]
    );

}




Eigen::Matrix3d EsekfomApi::getRotationMatrix() const 
{
    const state_ikfom state =kf_.get_x();

    return state.rot.toRotationMatrix();
}

state_ikfom EsekfomApi::getState() const 
{
    return kf_.get_x();
}


/*      getter */

/* 아직 안쓰는 함수 updateResidual() 지금은 껍데기만*/
void EsekfomApi::updateResidual(double lidar_point_cov, double& solve_time) 
{
    (void) lidar_point_cov;
    solve_time = 0.0;

    /*
        TODO: 
            LiDAR residual 단계에서 구현

            kf_.update_iterated_dyn_share_modified(lidar_point_cov, solve_time)들어가 예정.
    */
}


PoseState EsekfomApi::getPoseState() const 
{
    const state_ikfom kf_state = getState();

    PoseState pose_state;
    
    pose_state.pos = kf_state.pos;
    pose_state.vel = kf_state.vel;

    pose_state.rot = Eigen::Quaterniond(kf_state.rot.toRotationMatrix());
    pose_state.rot.normalize();

    pose_state.gyr_bias = kf_state.bg;
    pose_state.acc_bias = kf_state.ba;
    pose_state.gravity = kf_state.grav;

    pose_state.offset_R_L_I = kf_state.offset_R_L_I.toRotationMatrix();

    pose_state.offset_T_L_I = kf_state.offset_T_L_I;

    return pose_state;
}

void EsekfomApi::applyPoseCorrection(
    const Eigen::Matrix<double, 6, 1>& dx)
{
    if (!dx.allFinite())
    {
        std::cout << "[EsekfomApi::applyPoseCorrection][WARN] dx not finite"
                  << std::endl;
        return;
    }

    const Eigen::Vector3d dtheta = dx.head<3>();
    const Eigen::Vector3d dt = dx.tail<3>();

    const double rot_norm = dtheta.norm();
    const double trans_norm = dt.norm();

    if (rot_norm > 0.05 || trans_norm > 0.3)
    {
        std::cout << "[EsekfomApi::applyPoseCorrection][WARN] too large dx "
                  << " rot_norm=" << rot_norm
                  << " trans_norm=" << trans_norm
                  << " dx=" << dx.transpose()
                  << std::endl;
        return;
    }

    PoseState before = getPoseState();

    state_ikfom corrected_state = kf_.get_x();

    /*
        estimatePoseCorrection()의 dx 순서:
            dx.head<3>() = dtheta
            dx.tail<3>() = dt

        IKFoM state boxplus 순서:
            0~2   : pos
            3~5   : rot
            6~8   : offset_R_L_I
            9~11  : offset_T_L_I
            12~14 : vel
            15~17 : bg
            18~20 : ba
            21~22 : grav
    */
    Eigen::Matrix<double, 23, 1> dx_full =
        Eigen::Matrix<double, 23, 1>::Zero();

    dx_full.segment<3>(0) = dt;
    dx_full.segment<3>(3) = dtheta;

    corrected_state.boxplus(dx_full, 1.0);

    kf_.change_x(corrected_state);

    PoseState after = getPoseState();

    std::cout << "[EsekfomApi::applyPoseCorrection] "
              << " before_pos=" << before.pos.transpose()
              << " after_pos=" << after.pos.transpose()
              << " delta_pos=" << (after.pos - before.pos).transpose()
              << " rot_norm=" << rot_norm
              << " trans_norm=" << trans_norm
              << " dx=" << dx.transpose()
              << std::endl;
}