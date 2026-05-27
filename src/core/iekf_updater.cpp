#include "core/iekf_updater.hpp"



// IekfUpdater::IekfUpdater() 
// {

// }



namespace 
{
Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) 
{
    Eigen::Matrix3d m;
    m <<    0.0, -v.z(), v.y(),
            v.z(), 0.0, -v.x(),
            -v.y(), v.x(), 0.0;
    return m;
}
}

/*jacobian
    pose가 조금 바뀌면 residual이 얼마나 바뀌는가? 이걸 행렬로 만든 게 H다. 

*/
bool IekfUpdater::buildPoseJacobian(
    const State& state, 
    const ResidualCandidate& residual,
    Eigen::Matrix<double, 1,6>& H) const
{
    H.setZero();

    if(!residual.valid) return false;

    const Eigen::Matrix3d R = state.rotation.toRotationMatrix();
    const Eigen::Vector3d& pb = residual.point_body;
    const Eigen::Vector3d& n = residual.normal;

    const Eigen::Matrix3d pb_skew = skewSymmetric(pb);

    H.block<1,3>(0,0) = -n.transpose() * R * pb_skew;
    //rotation에 대한 Jacobian

    H.block<1,3>(0,3) = n.transpose();
    //translation에 대한 Jacobian

    return true;
}

/*
순서
    - 초기화
    - residuals 비어 잇는지 검사
    - for문으로 H_rows, residual_values들 모으기
    - 전체를 나타내는 H matrix, resisudal_vec 만들기
    - P와 R 만들기
    - Kalman Gain 계산
    - dx 계산
    - corrected_state 계산만 하기
    - rog 출력

*/
IekfUpdateResult IekfUpdater::update(
    const State& predicted_state,
    const std::vector<ResidualCandidate>& residuals,
    State& corrected_state) 
{
    IekfUpdateResult result;

    /*residual이 너무 적으면 update 자체 안하기. */
    static constexpr std::size_t MIN_IEKF_RESIDUALS = 80;

    result.residual_count = residuals.size();
    if (residuals.size() < MIN_IEKF_RESIDUALS)
    {
        corrected_state = predicted_state;
        result.updated = false;
        return result;
    }




    corrected_state = predicted_state;

    result.residual_count = residuals.size();


    if(residuals.empty())       return result;

    std::vector<Eigen::Matrix<double,1,6>> H_rows;
    std::vector<double> residual_values;
    std::vector<double> abs_residual_values;

    H_rows.reserve(residuals.size());
    residual_values.reserve(residuals.size());
    abs_residual_values.reserve(residuals.size());




    std::size_t jacobian_ok_count = 0;

 



    /*
        measurement noise.
        현재는 임시값이다.

        residual 단위가 meter이므로,
        sigma = 0.1m 라고 보면 variance = 0.01
    */



    /*
        너무 큰 correction이 튀는 것을 막기 위한 damping.
        처음 iEKF 연결 단계에서는 안정성을 위해 넣어둔다.

        여러 residual을 모아서 residual이 가장 작아지는 pose correction dx를 계산하는 1차 구현
        A * dx = -b이 핵심 식


        r + H * dx  ≈ 0이 되게 하는 dx를 찾는 것이다. 


    */






    /*하나의 residual condidate를 넣고 Jacobian 구함.*/
    for(const auto& r : residuals) 
    {
        if(!r.valid)    continue;

        Eigen::Matrix<double,1,6> H;
        
        /*poseJacobian생성 성공시 아래로 진행*/
        if(!buildPoseJacobian(predicted_state, r, H))
        {
            continue;
        }
        ++jacobian_ok_count;

        H_rows.push_back(H);
        residual_values.push_back(r.residual);
        abs_residual_values.push_back(r.abs_residual);
    }
    //valid residual마다 1 x 6 Jacobian H 하나 만들고, residual 값 하나 저장함. 


    /*H matrix와 residual vector 만들기 */
    if(jacobian_ok_count < MIN_IEKF_RESIDUALS)
    {
        corrected_state = predicted_state;
        result.updated = false;
        return result;
    }
    
    const int N = static_cast<int>(jacobian_ok_count);
    //실제 update에서 사용할 residual 개수

    Eigen::MatrixXd H(N,6);
    //전체 Jacobian들을 세로로 쌓은 값, 즉 H_i들이 모인것. 따라서 크기는 Nx6이다. 
    Eigen::VectorXd residual_vec(N);
    //전체 residual 값들을 세로로 쌓은 벡터 ... Nx1이다. 

    double sum_abs_residual = 0.0; //residual 절대값 평균 계산용
    double max_abs_residual = 0.0;  //가장 큰 residual 확인용

    /*값 복사해서 전체 H, 전체 residual 만들기*/
    for(int i =0 ; i< N ; ++i) 
    {
        H.row(i) = H_rows[static_cast<std::size_t>(i)];
        residual_vec(i) = residual_values[static_cast<std::size_t>(i)];

        const double abs_r = abs_residual_values[static_cast<std::size_t>(i)];

        sum_abs_residual += abs_r;

        /*현재까지 residual 중 가장 큰 값 써서, outlier 있는지 확인  */
        if(abs_r > max_abs_residual)
        {
            max_abs_residual = abs_r;
        }

    }

    result.mean_abs_residual = sum_abs_residual / static_cast<double>(N);

    result.max_abs_residual = max_abs_residual;


    /*
    P와 R 만들기 
        P: predicted_state를 얼마나 믿을지
        R: LiDAR residual을 얼마나 믿을지
        K : Kalman Gain
        dx : 최종 correction
    pose-

    */
   /*일단 pose 6DoF에 대한 임시 prior covariance를 부여*/
    constexpr double ROT_PRIOR_VAR = 1e-4;
    constexpr double POS_PRIOR_VAR = 5e-4; // 1e-3, 보수 5e-4
    constexpr double MEASUREMENT_VAR = 0.02; // 0.01 , 보수 0.02;

    Eigen::Matrix<double,6,6> P = Eigen::Matrix<double,6,6>::Zero();

    P.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * ROT_PRIOR_VAR;
    P.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * POS_PRIOR_VAR;
    // P : pose 불확실성
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(N,N) * MEASUREMENT_VAR;
    //R : LiDAR의 불확실성


    const Eigen::MatrixXd S = H * P *H.transpose() + R;
    //S는 pose 예측 때문에 생기는 residual 불확실성 + LiDAR 측정 자체의 불확실성
    const Eigen::Matrix<double, 6,Eigen::Dynamic> PHt = P * H.transpose();
    //Kalman gain 앞부분, pose state와 residual measurement가 얼마나 연결되어 있나?
    //residual이 변했을 때 pose를 어떤 방향으로 고쳐야 하는지 알려주는 연결 정보 


    const Eigen::MatrixXd S_inv = S.ldlt().solve(Eigen::MatrixXd::Identity(N,N));
    //S 역행렬,inverse()는 불안정해서, 이걸 주로 씀
    // residual 공간의 불확실성 행렬, S가 크면 residual 불확실 -> 덜반영

    const Eigen::Matrix<double,6,Eigen::Dynamic> K = PHt * S_inv;
    //Kalman gain
    const Eigen::Matrix<double,6,1> dx = K * (-residual_vec);


    result.dx = dx;
    result.dx_rot_norm = dx.head<3>().norm();
    result.dx_pos_norm = dx.tail<3>().norm();
    result.updated = true;


    /*corrected_state 계산*/
    const Eigen::Vector3d delta_rot = dx.head<3>();
    const Eigen::Vector3d delta_pos = dx.tail<3>();


    const Eigen::Quaterniond delta_q = smallAngleQuaternion(delta_rot);
    //rotation은 바로 못 더하니, quaternion으로 변환한다.
    // 여기서 작은 회전벡터가 변환됨. 

    corrected_state.rotation = predicted_state.rotation * delta_q;
    // 작은 회전 보정을  곱함

    corrected_state.rotation.normalize();
    //오차 줄이기 위해서,


    corrected_state.position = predicted_state.position + delta_pos;
    //위치는 그냥 더함

    const Eigen::Vector3d dx_pos_world = dx.tail<3>();

    const Eigen::Vector3d dx_pos_body = predicted_state.rotation.inverse() * dx_pos_world;
    //world기준을 body기준으로 변환,


    std::cout
        << "[IekfKalmanUpdate]"
        << " residuals=" << residuals.size()
        << " jacobian_ok=" << jacobian_ok_count
        << " mean_abs=" << result.mean_abs_residual
        << " max_abs=" << result.max_abs_residual
        << " P_rot_var=" << ROT_PRIOR_VAR
        << " P_pos_var=" << POS_PRIOR_VAR
        << " R_var=" << MEASUREMENT_VAR
        << " dx_body=("
        << dx_pos_body.x() << ", "
        << dx_pos_body.y() << ", "
        << dx_pos_body.z() << ")"
        << " dx_rot_norm=" << result.dx_rot_norm
        << " dx_pos_norm=" << result.dx_pos_norm
        << " updated=" << result.updated
        << std::endl;

    return result;


}

/*
    작은 회전 벡터 delta_theta를 quaternion으로  변환한다.

    이번 Jacobian은 
        R_new  = R * Exp(delta_theta) 형태의 right perturbation(오른쪽 곱) 기준이다. 
*/
Eigen::Quaterniond IekfUpdater::smallAngleQuaternion( const Eigen::Vector3d& delta_theta) const
{
    const double theta = delta_theta.norm();

    if(theta < 1e-12) 
    {
        Eigen::Quaterniond q (
            1.0,
            0.5 * delta_theta.x(),
            0.5 * delta_theta.y(),
            0.5 * delta_theta.z());
        q.normalize();
        return q;
    }

    const Eigen::Vector3d axis = delta_theta / theta;

    return Eigen::Quaterniond(Eigen::AngleAxisd(theta,axis));
}

