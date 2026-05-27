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

    /**
        이번 iEKF 브랜치에서는 pose 6DoF만 보정한다. 
        dx = [rotation_correction, position_correction]

        Kalman gain을 NxN으로 직접 만들지는 않고, 
        정보형 EKF / damped least squares 형태로 6x6 system을 만든다.

        A * dx = -b
     */

    Eigen::Matrix<double, 6, 6> A =
        Eigen::Matrix<double, 6, 6>::Zero();

    Eigen::Matrix<double, 6, 1> b =
        Eigen::Matrix<double, 6, 1>::Zero();

    std::size_t jacobian_ok_count = 0;

    double sum_abs_residual = 0.0;
    double max_abs_residual = 0.0;



    /*
        measurement noise.
        현재는 임시값이다.

        residual 단위가 meter이므로,
        sigma = 0.1m 라고 보면 variance = 0.01
    */

    constexpr double MEASUREMENT_VAR = 0.01;
    constexpr double INV_MEASUREMENT_VAR = 1.0 / MEASUREMENT_VAR;

    /*
        너무 큰 correction이 튀는 것을 막기 위한 damping.
        처음 iEKF 연결 단계에서는 안정성을 위해 넣어둔다.

        여러 residual을 모아서 residual이 가장 작아지는 pose correction dx를 계산하는 1차 구현
        A * dx = -b이 핵심 식


        r + H * dx  ≈ 0이 되게 하는 dx를 찾는 것이다. 


    */
    constexpr double DAMPING = 10;
    //A행렬이 너무 불안정하거나 역행렬 풀기 어려울 때 대각선에 작은 값 더하여 계산 안정화
    //값이 클수록 correction이 더 작아짐.





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

        const double residual = r.residual;


        A += H.transpose() * H * INV_MEASUREMENT_VAR;
        b += H.transpose() * residual * INV_MEASUREMENT_VAR;

        sum_abs_residual += r.abs_residual;

        if(r.abs_residual > max_abs_residual) 
        {
            max_abs_residual = r.abs_residual;
        }

    }

    if(jacobian_ok_count ==0)  return result;
    
    A += DAMPING * Eigen::Matrix<double,6,6>::Identity();

    const Eigen::Matrix<double,6,1>  dx = - A.ldlt().solve(b);

    result.dx = dx;
    result.dx_rot_norm = dx.head<3>().norm();
    result.dx_pos_norm = dx.tail<3>().norm();

    result.mean_abs_residual = sum_abs_residual / static_cast<double>(jacobian_ok_count);

    result.max_abs_residual = max_abs_residual;

    /*
        correction 적용.
            현재 Jacobian은 right perturbation 기준이므로 
            rotation은 R_new * R * Exp(dtheta)형태로 적용함. 
    */
   const Eigen::Vector3d delta_rot= dx.head<3>();
   const Eigen::Vector3d delta_pos =dx.tail<3>();

   const Eigen::Quaterniond delta_q = smallAngleQuaternion(delta_rot);

   corrected_state.rotation = predicted_state.rotation * delta_q;

   corrected_state.rotation.normalize();

   corrected_state.position = predicted_state.position + delta_pos;


   result.updated = true;


const Eigen::Vector3d dx_pos_world = dx.tail<3>();
const Eigen::Vector3d dx_pos_body =
    predicted_state.rotation.inverse() * dx_pos_world;

std::cout
    << "[IekfCorrectionDir]"
    << " dx_body=("
    << dx_pos_body.x() << ", "
    << dx_pos_body.y() << ", "
    << dx_pos_body.z() << ")"
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

