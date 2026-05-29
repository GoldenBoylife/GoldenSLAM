#pragma once
#include <Eigen/Dense>
#include <use-ikfom.hpp>
#include <third_party/IKFoM_toolkit/esekfom/esekfom.hpp>

// iEKF 상태 추정기 — esekfom::esekf 를 감싸는 클래스
// 상태 추정 엔진
// state: pos, rot, extrinsic R/T, vel, gyro bias, acc bias, gravity  (DOF=23)
// input: acc, gyro  (12-dim noise)
class IEkf
{
public:
    using EkfType = esekfom::esekf<state_ikfom, 12, input_ikfom>;
    using CovType = Eigen::Matrix<double, 23, 23>;
    using HModel  = void (*)(state_ikfom&, esekfom::dyn_share_datastruct<double>&);
    //인자 2개를 받고, 리턴은 없는 함수의 주소 타입 alias
    //HModel : 측정 모델 함수 포인터 타입



    IEkf();

    // max_iterations: iEKF 최대 반복 횟수, h_model: 측정 모델 콜백 (raw function pointer)
    void init(int max_iterations, HModel h_model);
    //호출 함수 2개 필요.
    // 풀어서 쓰면, void init(int max_iterations, void (*h_model)(state_ikfom&, esekfom::dyn_share_datastruct<double>&); 이런 모양됨
    //즉 함수 포인터를 두번째에 넣으라는 소리임. 


    // IMU propagation — esekfom 내부에서 Q/in 을 non-const ref 로 받음
    void predict(double dt, Eigen::Matrix<double, 12, 12>& Q, input_ikfom& in);

    // iEKF iterative measurement update (correction step)
    void update(double laser_point_cov, double& solve_time);

    // State access — esekfom getter 가 non-const 이므로 const 제거
    state_ikfom get_x();
    CovType     get_P();
    void        change_x(state_ikfom x);
    void        change_P(CovType P);

private:
    EkfType kf_;
    double  epsi_[23];  // convergence threshold per state DOF
};
