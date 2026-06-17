#ifndef USE_IKFOM_H
#define USE_IKFOM_H

#include <cmath>
#include <Eigen/Dense>

#include "third_party/IKFoM_toolkit/esekfom/esekfom.hpp"

/*
    이 파일은 third_party 라이브러리가 아니다. 

    역할:
    1. IKFoM/esekfom 엔진에서 사용한 FAST-LIO2용 state정의
    2. IMU input 정의
    3. process noise 정의
    4. prediction model get_f()
    5. Jacobian df_dx(), df_dw()정의

    esekfom.hpp = iEKF 엔진
    use-ikfom.hpp = FAST-LIO2용 상태/입력/동영학 모델
    esekfom_api.hpp = wrapper
*/

typedef MTK::vect<3,double> vect3;
typedef MTK::SO3<double> SO3;
typedef MTK::S2<double, 98090,10000,1> S2;
typedef MTK::vect<1,double> vect1;
typedef MTK::vect<2,double> vect2;


/*
    FAST-LIO2에서 사용하는 state
    pos : world 기준 위치
    rot : world 기준 IMU 자세
    offset_R_L_I  : LiDAR to IMU 회전 extrinsic
    offset_T_L_I : LiDAR to IMU 이동 extrinsic
    vel     : world 기준 속도
    bg :    gyro bias
    ba :    acc bias
    grav    : gravity

*/
MTK_BUILD_MANIFOLD(state_ikfom,
((vect3, pos))
((SO3, rot))
((SO3, offset_R_L_I))
((vect3, offset_T_L_I))
((vect3, vel))
((vect3, bg))
((vect3, ba))
((S2, grav))
);


/*
    IMU prediction input    

        acc : 선가속도
        gyro : 각속도
*/
MTK_BUILD_MANIFOLD(input_ikfom,
    ((vect3, acc))
    ((vect3, gyro))
);


/* 
    process noise
         ng : gyro noise
         na : acc noise
         nbg : gyro bias noise
         nba : acc bias noise

    */
MTK_BUILD_MANIFOLD(process_noise_ikfom,
    ((vect3, ng))
    ((vect3, na))
    ((vect3,nbg))
    ((vect3, nba))

);

/*
    process_noise_cov()
        IMU propagation 때 들어가는 process noise covariance Q를 만드는 함수
        즉,
        IMU값은 완벽하지 않다. 
        gyro, acc, 노이즈가 끼어있고, 
        시간이 지날수록 gyro bias, acc bias도 조금씩 변한다.
        그 불확실성을 EKF로 알려주는 행렬이 Q다.

        이 함수가 만드는 것은 12x12 noise covariance다.
        imuProcessor에서 Q를 만들때 쓴다.
 */
inline MTK::get_cov<process_noise_ikfom>::type process_noise_cov()
{
    MTK::get_cov<process_noise_ikfom>::type cov = 
        MTK::get_cov<process_noise_ikfom>::type::Zero();

    
    /*대각선에 값 부여*/
    MTK::setDiagonal<process_noise_ikfom, vect3, 0> (
        cov, &process_noise_ikfom::ng, 0.0001);
    
    MTK::setDiagonal<process_noise_ikfom, vect3,3>(
        cov,&process_noise_ikfom::na, 0.0001);
    MTK::setDiagonal<process_noise_ikfom, vect3,6> (
        cov,&process_noise_ikfom::nbg, 0.00001);

    MTK::setDiagonal<process_noise_ikfom,vect3,9>(
        cov,&process_noise_ikfom::nba, 0.00001);
    
    return cov;
}
/*
    prediction model f(x,u)
        imu값으로 state가 어떻게 변하는지 정의한다.
        get_f(state, imu_input)
            현재 state와 imu_input을 넣으면 state가 시간에 따라
            어떻게 변하는지 반환함. 

            //dot : 시간에 대한 미분되었다는 뜻
        pos_dot = vel
        rot_dot = gyro - bg
        vel_dot = R * (acc - ba) + gravity

        res[0:3]    = pos_dot = vel
        res[3:6]    = rot_dot = gyro - bg
        res[6:9]    = offset_R_L_I_dot = 0
        res[9:12]   = offset_T_L_I_dot = 0
        res[12:15]  = vel_dot = R(acc - ba) + gravity

        res[15:18]  = bg_dot = 0
        res[18:21]  = ba_dot = 0
        res[21:24]  = grav_dot = 0
        → 24 x 1 state 미분값 계산(순간 변화율들)

*/
inline Eigen::Matrix<double,24,1> get_f(state_ikfom& s, const input_ikfom& in)
{
    Eigen::Matrix<double, 24,1> res = Eigen::Matrix<double, 24,1>::Zero();
    //반환값이 state 변화량 벡터로 state가 어떻게 변하는지 뜻함

    vect3 omega;
    in.gyro.boxminus(omega, s.bg);
    //omega = gyro - gyro bias


    vect3 a_inertial = s.rot * (in.acc - s.ba);
    //acc bias 제거후 world좌표계로 변환


    for(int i =0; i< 3; i++) 
    {
        res(i) = s.vel[i];
        res(i+3) = omega[i];
        res(i+12) = a_inertial[i] + s.grav[i];
        // gravity에 더함
        //res(i+12) = vel_dot 임
    }
    return res;
}


/*
    state Jacobian df/dx
    row 24개는 전부 state 미분값
        row 0~2    : pos_dot
        row 3~5    : rot_dot
        row 6~8    : offset_R_L_I_dot
        row 9~11   : offset_T_L_I_dot
        row 12~14  : vel_dot
        row 15~17  : bg_dot
        row 18~20  : ba_dot
        row 21~23  : grav_dot
    col 23개는 state error값
        col 0~2    : pos error
        col 3~5    : rot error
        col 6~8    : offset_R_L_I error
        col 9~11   : offset_T_L_I error
        col 12~14  : vel error
        col 15~17  : bg error
        col 18~20  : ba error
        col 21~22  : grav error
*/

inline Eigen::Matrix<double, 24,23> df_dx(state_ikfom& s, const input_ikfom& in) 
{
    Eigen::Matrix<double,24,23> cov = Eigen::Matrix<double, 24,23>::Zero();

    //col 0~14(vel err)오차가 row 0~2(pos_dot)에 어떤 영향 주는지



    cov.template block<3,3>(0,12) = Eigen::Matrix3d::Identity();
    // pose = vel, 위치의 변화율은 속도
    // row 0~2가  col 12~14에 들어감.    

    vect3 acc_;
    in.acc.boxminus(acc_, s.ba);
    //bias 제거한 가속도 , acc_ =  input.acc - bias

    vect3 omega;
    in.gyro.boxminus(omega, s.bg);

    cov.template block<3,3>(12,3)  =  - s.rot.toRotationMatrix() * MTK::hat(acc_);
    // row 12~14 : vel_dot
    // col 3~5 : rot_err
    // rot_err가 vel_dot에 어떤 영향 주는가?
    // vel_dot = R * acc + gravity에서, 
    // s.rot.toRotationMatrix() : R은 회전행렬임., 현재 state의 자세를 3x3행렬로 바꾼것, 이걸 통해서 body 기준에서 world기준으로 바꿔줌.
    //vel_dot = body 좌표계에서 
    // acc_ : body frame 가속도이고, s.bs가 삭제된 가속도다.
    //이때 vel은 world frame기준이므로, R*acc_ 해서  world frame으로 바꾼다. 
    
    //MTK::hat() : 회전/각속도/외적을 행렬로 표현하기 위한 도구
    //hat(acc_) 벡터 acc_를 외적 행렬로 바꿔줌


    /* 
        acc bias error 가 vel_dot에 주는 영향이다. 

        vel_dot = R *(acc -ba) + gravity
        ba가 커지면 vel_dot은 -R 방향으로 변한다.
    
    */
    cov.template block<3,3>(12,18) =  -s.rot.toRotationMatrix();


    /*
        gravity error가 vel_dot에 주는 여향

        grav는 minifold라서 error 차원이 2개다. 
        그래서 block 크기는 3x2다.

     */

     Eigen::Matrix<state_ikfom::scalar, 2,1> vec = Eigen::Matrix<state_ikfom::scalar,2,1>::Zero();

     Eigen::Matrix<state_ikfom::scalar,3,2> grav_matrix; 

     s.S2_Mx(grav_matrix,vec,21);

     cov.template block<3,2>(12,21) =  grav_matrix;

     /*
        rot_dot = gyro - bg
        
        gyro bias eror 가 rot_dot에 주는 영향이다. 
        bg가 커지면, rot_dot은 음의 방향으로 변한다.
     */
    cov.template block<3,3>(3,15) = -Eigen::Matrix3d::Identity();

    return cov;
}

/*
    noise Jacobian df/dw

    ros 24개: state 미분값 방향

    col 12개:
        nose state
        col 0~2 : gyro noise ng
        col 3~5 : acc noise na
        col 6~8 gyro bias noise nbg
        col 9~11 : acc bias noise nba
*/
inline Eigen::Matrix<double,24,12> df_dw(state_ikfom& s, const input_ikfom& in)
{
    (void) in;
    Eigen::Matrix<double,24,12> cov =  Eigen::Matrix<double,24,12>::Zero();

    /*
        vel_dot = R* acc
        acc noise 가 vel_dot에 들어가는 방향이다. 
        acc_noise는 body_frame 기준으로 world_frame기준으로 가려면 R붙어야 함. 
        본문 FAST-LIO2에서는 -R 형태로 들어간다. 

    */
   cov.template block<3,3>(12,3) = -s.rot.toRotationMatrix();

   /* 
   rot_dot = gyro -bg
   gyro noise 가 rot_dot에 들어가는 영향이다.
   */

    cov.template block<3,3>(3,0) = -Eigen::Matrix3d::Identity();

    /*
        gyro bias random walk noise

    */
   cov.template block<3,3>(15,6) = Eigen::Matrix3d::Identity();


   /*   
        acc bias random walk noise
    */
   cov.template block<3,3>(18,9) = Eigen::Matrix3d::Identity();

   return cov;

}




#endif// USE_IKFOM_H