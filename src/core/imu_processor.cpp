#include "core/imu_processor.hpp"



ImuProcessor::ImuProcessor()
{
    initialized_= false;
    gravity_  = Eigen::Vector3d(0.0, 0.0, -9.80665);
    init_frame_count_ = 0;
    init_sample_count_ = 0;
    mean_acc_ = Eigen::Vector3d::Zero();
    mean_gyr_ = Eigen::Vector3d::Zero();
    
    acc_scale_ = 1.0;

}
/*measureGroup 안에 들어온 imu값보고 IMU processor를 초기화함.*/
bool ImuProcessor::initImuProcessor(const MeasureGroup& meas, State& state)
{
    /*
    순서
        초기 몇개 LiDAR frame 동안 imu를 모은다.
        ->
        mean_acc 게산
        mean_gyr 계산
        ->
        gyr_bias = mean_gyr
        acc_bias = 0
        gravity 방향 기준으로 초기 rotation 정렬
        ->
        initialitzed = true;
    
    */
    /* imu 값 없으면 skip*/
    if(meas.imus.empty()) 
    {
        std::cout << "[ImuProcessor] init skip : empty imu" << std::endl;
        return false;
    }

    ++init_frame_count_;

    /*incremental mean*/
    for(const auto& imu : meas.imus) 
    {
        const Eigen::Vector3d acc_body = getAcc(imu);
        const Eigen::Vector3d gyr_body = getGyr(imu);
        
        ++init_sample_count_;

        /*incremental mean*/
        mean_acc_ += (acc_body - mean_acc_) / static_cast<double>(init_sample_count_);
        mean_gyr_ += (gyr_body - mean_gyr_) / static_cast<double>(init_sample_count_);
    }
    // std::cout
    // << "[ImuProcessor::init]"
    // << " frame=" << init_frame_count_
    // << " sample=" << init_sample_count_
    // << " mean_acc=" << mean_acc_.transpose()
    // << " acc_norm=" << mean_acc_.norm() //백터 길이가 9.8로 정상인지 확인하기 위해서
    // << " mean_gyr=" << mean_gyr_.transpose()
    //전치 행렬로 1행으로 만든다. 
    // << std::endl;

    /*약 2초 모은다.*/
    if(init_frame_count_ < INIT_FRAME_NUM) return false;

    const double last_time = getImuTime(meas.imus.back());
    state.timestamp = last_time;

    scaleGravity();

    //정지 상태에서 평균 gyro 는 gyro bias로 봄
    state.gyr_bias = mean_gyr_;
    state.acc_bias = Eigen::Vector3d::Zero(); 
    //acc평균에는 중력이 들어 있기 때문에 mean_acc를 그대로 넣으면 안됨.  나중에 iEKF 이후에 추정할것임. 
    state.position = Eigen::Vector3d::Zero();
    state.velocity = Eigen::Vector3d::Zero();

    /*gravity_ 가 0,0,-9.8로 가정*/
    const Eigen::Vector3d target_acc_world = -gravity_;
    //target_acc_world = (0,0,+9.8)됨.


    /*mean_acc 방향을 world의 +z방향, 즉 -gravity 방향으로 맞춤*/
    // 초기화 때 acc 평균 방향을 이용해서 초기 roll /pitch를 잡음.
    // IMU body frame에서 측정된 mean_acc방향을 world frame의 위쪽 방향(-gravity)로 맞추는 회전값 구함
    // IMU 가 정지상태에서 느끼는 "위쪽 방향"을 world 좌표계에서 "위쪽 방향"과 맞추는 초기회전값 만든다.
    // normalized() 크기버리고 방향만 남긴 단위벡터로 만듬.
    //  (1.0, 0.0, 9.7)
    //      ↓ normalized
    //  (0.102, 0.0, 0.995)

    // FromTwoVectors(_,_) : 첫번째 벡터 방향을 두번째 벡터 방향으로 돌리는 회전값 만들기
    state.rotation = 
        Eigen::Quaterniond::FromTwoVectors(
            mean_acc_.normalized(),
            target_acc_world.normalized()

        ).normalized();
    
    initialized_ = true;


    std::cout
        << "[ImuProcessor] initialized"
        << " time=" << std::fixed << std::setprecision(9) << state.timestamp
        << " gyr_bias=" << state.gyr_bias.transpose()
        << " acc_bias=" << state.acc_bias.transpose() 
        << " mean_acc_norm=" << mean_acc_.norm()
        << " acc_scale=" << acc_scale_
        << " gravity=" << gravity_.transpose()
        << " init_rot(xyzw)="
        << state.rotation.x() << ", "
        << state.rotation.y() << ", "
        << state.rotation.z() << ", "
        << state.rotation.w()
        << std::endl;

    return true;

}

bool ImuProcessor::isInitialized() const
{
    return initialized_;
}




/*imu_propagate*/
// IMU 값 2개 사이의 시간 간격 dt 동안, 현재 state가 어떻게 변했을지를 적분해서 다음 state를 예측하는 코드
void ImuProcessor::propagate(const MeasureGroup& meas, State& state,ImuPropagatedPoseHistory& pose_history)
{
    pose_history.clear();
    /*초기화 안되어 있으면 초기화*/
    if(!initialized_) 
    {
        if(!initImuProcessor(meas,state))   return;
    }

    /*imu processor 초기화됨*/
    if(meas.imus.size() <2) 
    {
        std::cout << "[ImuProcessor] too free imu, count = " << meas.imus.size() << std::endl;
        return;
    }

    for(std::size_t i =0 ; i+1 < meas.imus.size(); ++i)
    {
        const auto& imu_curr = meas.imus[i];
        const auto& imu_next = meas.imus[i+1];
        
        const double t_curr = getImuTime(imu_curr);
        const double t_next = getImuTime(imu_next);
        const double dt = t_next - t_curr;

        if(dt <= 0.0 || dt > 0.02)
        {
             std::cout
                << "[ImuProcessor] invalid dt"
                << " dt=" << dt
                << std::endl;
            continue;
        }        
    /*body기준 좌표계에서의 gyro와 acc*/
    //imu body frame 기준 각속도와 속도
    const Eigen::Vector3d gyr_body = getGyr(imu_curr);

    // raw acc는 getAcc()에서 받은 원본 m/s^2 값
    const Eigen::Vector3d acc_body_raw = getAcc(imu_curr);

    // 초기 mean_acc_norm 기준으로 gravity 크기에 맞춘 acc
    const Eigen::Vector3d acc_body = acc_body_raw * acc_scale_;
    // std::cout
    //     << "[ImuProcessor] imu"
    //     << " dt=" << std::fixed << std::setprecision(9) << dt
    //     << " gyro=("
    //     << gyro_body.x() << ", "
    //     << gyro_body.y() << ", "
    //     << gyro_body.z() << ")"
    //     << " acc=("
    //     << acc_body.x() << ", "
    //     << acc_body.y() << ", "
    //     << acc_body.z() << ")"
    //     << " acc_norm=" << acc_body.norm()
    //     << std::endl;

    /*0. 센서 오차 bias값 제거*/
    const Eigen::Vector3d gyr_unbias = gyr_body - state.gyr_bias;
    //state_gyr_bias는 mean_gyr가 있으니,  센서가 읽은 gyro값에서 고정 오차를 뺀 실제 회전에 가까운 값이다.

    const Eigen::Vector3d acc_unbias = acc_body - state.acc_bias;
    //

    /*1. gyro 적분 -> rotation 예측*/
    //각속도*시간 = 회전량, deltaQ()로 회전량을 quaternion으로 바꿔줌 그걸 기존 rotation에 곱함
    const Eigen::Quaterniond delta_q = deltaQ(gyr_unbias,dt);
    state.rotation = (state.rotation * delta_q).normalized();
    //quaternion은 원래 길이가 1이어야 정상적인 회전이다, 반복하다보면 조금씩 오차가 쌓인다.
    // 매번 길이를 1로 만들기 위해서 normalized()한다.

    
    /*2. acc body frame*/
    const Eigen::Vector3d acc_world = state.rotation * acc_unbias + gravity_;
    //body기준acc -> world기준acc로 
    // 실제 world acc구하기 위해서 gravity를 더해준다.
    // R * acc_body ≈ (0, 0, +9.8)
    // gravity     ≈ (0, 0, -9.8)
    // acc_world ≈ (0, 0, 0)  이제 실제 움직이는 world기준 acc가 나온다. 


    /*3. velocity / position적분*/
    //acc_world로 속도와 위치를 예측가능
    //오차가 심해져서 나중에 LiDAR IEKF update가 필요 
    state.position += state.velocity * dt + 0.5 * acc_world * dt*dt;
    state.velocity +=acc_world *dt;

    state.timestamp = t_next;
    //이미 행동이 이뤄진 상태에서의 시간이 필요하다. 그래서 t_next 넣음. 

    pose_history.push_back({
        state.timestamp,
        state.rotation,
        state.position,
        state.velocity
    });

// std::cout
//     << "[propagate]"
//     << " dt=" << dt
//     << " acc_raw=" << acc_body_raw.transpose()
//     << " acc_scaled=" << acc_body.transpose()
//     << " acc_scale=" << acc_scale_
//     << " acc_unbias=" << acc_unbias.transpose()
//     << " acc_world=" << acc_world.transpose()
//     << " vel=" << state.velocity.transpose()
//     << " pos=" << state.position.transpose()
//     << std::endl;
    
    } //for



}


double ImuProcessor::getImuTime(const sensor_msgs::msg::Imu::ConstSharedPtr& imu) const
{
    return static_cast<double>(imu->header.stamp.sec) + static_cast<double>(imu->header.stamp.nanosec) *1e-9;
}



Eigen::Vector3d ImuProcessor::getGyr(const sensor_msgs::msg::Imu::ConstSharedPtr& imu) const
{
    return Eigen::Vector3d(
        imu->angular_velocity.x,
        imu->angular_velocity.y,
        imu->angular_velocity.z
    );
}


Eigen::Vector3d ImuProcessor::getAcc(const sensor_msgs::msg::Imu::ConstSharedPtr& imu) const
{
    constexpr double G = 9.80665; //중력

    return G * Eigen::Vector3d(
        imu->linear_acceleration.x,
        imu->linear_acceleration.y,
        imu->linear_acceleration.z
    );

}

Eigen::Quaterniond ImuProcessor::deltaQ(const Eigen::Vector3d& omega, double dt) const
{
    const Eigen::Vector3d delta_angle = omega * dt;
    const double angle = delta_angle.norm();

    if(angle < 1e-12) 
    {
        return Eigen::Quaterniond::Identity();
    }

    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, delta_angle.normalized()));
}

void ImuProcessor::scaleGravity() 
{
    constexpr double G = 9.80665;
    const double acc_norm = mean_acc_.norm();

    if(acc_norm < 1e-6) 
    {
        std::cout << "[ImuProcessor] init failed : invalid acc norm" << std::endl;
        return ;
    }
    // mean_acc 크기를 표준 중력 크기에 맞추기 위한 scale
    // 예: mean_acc.norm() = 9.75라면 acc_scale_ = 9.80665 / 9.75
    acc_scale_ = G / acc_norm;
}

/*      imu_propagate*/