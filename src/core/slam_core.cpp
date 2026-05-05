#include "core/slam_core.hpp"

SlamCore::SlamCore() 
{
    std::cout << "SlamCore started" << std::endl;
}
SlamCore::~SlamCore()
{

}


double SlamCore::toSec(const builtin_interfaces::msg::Time& stamp) const
{
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

bool SlamCore::syncMeasureGroup(MeasureGroup& meas)
{
    /*
    순서
        mutex lock
        버퍼에 아무것도 없으면 준비가 안되어 있으니, return ;

        처음에는 lidar_pushed가 false라 올수 있고, 
        현재 처리 대상으로 잡아둔 lidar스캔이 아직 하나 없으면 골라 잡음. 

        if()
    */
    std::lock_guard<std::mutex> lock(buffer_mutex_);

   if(!hasSensorBuffer())               return false;
   if(!prepareLidarFrame(meas))         return false;
   if(!hasEnoughImuForCurrentLidarFrame())   return false;

    std::cout
        << "[debug sync] cloud="
        << (meas.lidar_frame.cloud ? "ok" : "null")
        << ", end=" << meas.frame_end_time
        << std::endl;



   collectImuForCurrentLidarFrame(meas);
   consumeCurrentLidarFrameBuffer(); //현 프레임 삭제

   return true;
}


bool SlamCore::hasSensorBuffer() const 
{
    // return !lidar_buffer_.empty() && !imu_buffer_.empty();
    return !lidar_frame_buffer_.empty() && !imu_buffer_.empty();
}

/*현재 처리 대상으로 잡아둔 LiDAR 프레임을 준비하고 끝시간까지 계산*/
//FIFO queue 기반 버퍼링 구조
bool SlamCore::prepareLidarFrame(MeasureGroup& meas)
{
    /*처음 일때 들어오기
     현재 처리 대상으로 잡아둔 LiDAr스캔이 아직 없으면 하나 골라서 대상으로 하기.
     lidar는 하나를 대상으로 잡아두었는데, 한 frame을 맞추려면 IMU값도 기다려야 함. 
    */
    if(lidar_frame_buffer_.empty())
        return false; 
    
    meas.lidar_frame = lidar_frame_buffer_.front();

    /*buffer가 비워졌음 그래서 채워질 준비가됨*/
    if(!lidar_pushed_)
    {

        meas.lidar_frame = lidar_frame_buffer_.front();   

        //너무 point size가 작으면(비정상)
        if(meas.lidar_frame.cloud->points.size() <= 1) 
        {
            lidar_end_time_ = meas.lidar_frame.frame_beg_time + lidar_mean_scantime_;
            //대략적으로 end_time을 넣음.
        }
        else 
        {
            /*사이즈 괜찮고, 마지막 점의 상대시간 값이 너무 작을 때(비정상), 평소의 평균값으로 떼우기*/
            const double scan_time = 
                meas.lidar_frame.cloud->points.back().relative_time;
                //마지막원소의 relative_time을 씀
            
            if(scan_time < 0.5 * lidar_mean_scantime_) 
            {
                lidar_end_time_ = meas.lidar_frame.frame_beg_time + lidar_mean_scantime_;
            }
            else 
            {
                /*정상: 사이즈 좋고, 마지막점 상대시간도 적당함*/
                ++scan_num;
                lidar_end_time_ = meas.lidar_frame.frame_beg_time + scan_time;
                lidar_mean_scantime_  +=  (scan_time - lidar_mean_scantime_) /scan_num;
                //평균적으로 한 스캔이 얼마나 걸리는지 학습
                std::cout
                    << "[prepareLidarFrame] "
                    << "beg=" << meas.lidar_frame.frame_beg_time
                    << ", scan_time=" << scan_time
                    << ", end=" << lidar_end_time_
                    << ", mean_scan=" << lidar_mean_scantime_
                    << std::endl;            }
                    //정상이라면 scan_time은 0.0988, lidar_mean_scantime_ 약 0.1
        }
        lidar_pushed_ = true;
    }
        meas.frame_end_time = lidar_end_time_;

    return  true;
            
}
/*한 프레임이 준비완료되었는지 확인*/
bool SlamCore::hasEnoughImuForCurrentLidarFrame() const
{
    return last_timestamp_imu_ >= lidar_end_time_;
    //현 lidar frame 끝시간인 lidar_end_time_까지 imu가 들어와야 하니까. 
}

/*현 lidar_frame 끝시간 이하의 imu값들 모으기*/
//imu_buffer가 들어온 시점이 이번 frame인지 다음 frame인지 모르니까. 
//일단은 imu_buffer에 쌓아 놓고, 조건에 맞추어서 meas.imus에 넣는다. 
//전제: imu의 timestamp값이 오름차순 정렬되어 있다. 
// 현 상태에서 시간정렬 안된상태로 들어온다면, 이 코드는 무너진다.
// 따라서 시간 역전이 발생한다면, 그 버퍼는 clear해서 버려버리고 다시 시작한다. 
void SlamCore::collectImuForCurrentLidarFrame(MeasureGroup& meas)
{
    meas.imus.clear();

    while(!imu_buffer_.empty() )
    {
        const double  imu_time = toSec(imu_buffer_.front()->header.stamp);
        if(imu_time > lidar_end_time_) break;
        //만약 다음 frame꺼가 온다면, imu값을 buffer에 남겨둔다.그리고 

        meas.imus.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();

    }
}

/*현 lidar의 frame 삭제*/
void SlamCore::consumeCurrentLidarFrameBuffer()
{
    if(!lidar_frame_buffer_.empty()) 
    {
        lidar_frame_buffer_.pop_front();
    }
    lidar_pushed_ =false;
}


void SlamCore::processMeasure(const MeasureGroup& meas)
{
    if(!isValidMeasure(meas)) return;
    //입력 묶음이 최소 조건 만족?

    /*아직 초기화 안되었을때*/
    if(!imu_initialized_)
    {
        initializeImu(meas); 
        return;
    }

    runFrontend(meas);
    //초기화 끝난 뒤에 진짜 frontend처리
}

bool SlamCore::isValidMeasure(const MeasureGroup& meas) const
{
    if(!meas.lidar_frame.cloud)  return false;
    if(meas.lidar_frame.cloud->points.empty()) return false;
    if(meas.imus.empty()) return false;
    if(meas.frame_end_time <= meas.lidar_frame.frame_beg_time) return false;

    return true;
}

/*
시작 직후 IMU 상태가 안정적이지 않을 수 있으니, 
20개 프레임 동안은 propatation, deskew안하고 초기 추정위한 데이터로 쓴다.
*/
void SlamCore::initializeImu(const MeasureGroup& meas) 
{

    if(!isValidImu(meas)) return;
    accumulateImuInitStats(meas);
    logImuInitStatus();

    if(init_frame_count_ < 20) return; //ready to imuInitState.
    finalizeImuInitStats();

}
bool SlamCore::isValidImu(const MeasureGroup& meas) const
{
    /*false 조건
        1. imu 비어있으면 
        2. lidar_frame의 cloud가 없을때
        3. lidar_frame의 객체인 cloud는 있는데, 점이 없을때
    
    */
    if(meas.imus.empty()) return false;
    if(!meas.lidar_frame.cloud)  return false;
    if(meas.lidar_frame.cloud->points.empty()) return false;

    return true;
}

/*초기화 통계 누적*/
// 프레임 수가 눌고, 평균 acc, 평균 gyr 구하기.
void SlamCore::accumulateImuInitStats(const MeasureGroup& meas)
{
    ++init_frame_count_;
    imu_count_in_frame_ = meas.imus.size();
    for(const auto& imu_msg : meas.imus) 
    {
        Eigen::Vector3d acc(
            imu_msg->linear_acceleration.x,
            imu_msg->linear_acceleration.y,
            imu_msg->linear_acceleration.z
        );

        Eigen::Vector3d gyr(
            imu_msg->angular_velocity.x,
            imu_msg->angular_velocity.y,
            imu_msg->angular_velocity.z
        );

        ++init_imu_sample_count_;
        mean_acc_ +=(acc - mean_acc_) / static_cast<double>(init_imu_sample_count_);
        mean_gyr_ +=(gyr - mean_gyr_) / static_cast<double>(init_imu_sample_count_);
        
    }
}
void SlamCore::finalizeImuInitStats()
{
    imu_initialized_ = true;

    std::cout << "[init_imu] done |" 
              <<  "mean_acc=(" << mean_acc_.x() << ", "
                               << mean_acc_.y() << ", "
                               << mean_acc_.z() <<")"
              <<  "mean_gyr=(" << mean_gyr_.x() << ", "
                               << mean_gyr_.y() << ", "
                               << mean_gyr_.z() <<")"
              << std::endl;
            
}


void SlamCore::logImuInitStatus()
{
    std::cout 
        << "init_frame_count_ : " << init_frame_count_ 
        << ", imu_count_in_frame_ : "  << imu_count_in_frame_ 
        << ", total_imu_samples_ : " << init_imu_sample_count_
        <<", mean_acc=(" << mean_acc_.x() << ", " 
                            << mean_acc_.y() << ", "
                            << mean_acc_.z() << ", "
        <<", mean_gyr=(" << mean_gyr_.x() << ", " 
                            << mean_gyr_.y() << ", "
                            << mean_gyr_.z() << ")"
    << std::endl;
    //이것을 통해서 IMU초기화가 제대로 수렴하고 있는지 볼 수있다. 
}

/*순서 잡아주는*/
void SlamCore::runFrontend(const MeasureGroup& meas)
{
    debug_preprocess_cloud_ = meas.lidar_frame.cloud;
    last_processed_frame_time_ = meas.frame_end_time;

    predictByImu(meas); //현재 위치 예측 하기 by IMU 적분
    
    /**debugging */
    std::cout
        << "[predictByImu] deskew pose count = "
        << deskew_poses_.size()
        << std::endl;


    if (!deskew_poses_.empty()) {
        const auto& last_pose = deskew_poses_.back();
        std::cout
            << "[predictByImu] last deskew pose | "
            << "offset=" << last_pose.offset_time
            << ", pos=(" << last_pose.pos.x() << ", "
                        << last_pose.pos.y() << ", "
                        << last_pose.pos.z() << ")"
            << std::endl;
    }


    undistortLidar(meas);
}

/*이전 상태를 가지고
IMU 측정값을 시간 순으로 적분하여 , 다음 상태로 앞으로 밀어 넣음. 
이전상태  -> IMU적분 -> 예측된 현재 상태
predictByImu(meas)
 ├─ 이번 프레임에 쓸 IMU 시퀀스 준비
 ├─ IMU 두 개씩 묶어서 구간별 평균 값 계산
 ├─ dt 계산
 ├─ 현재 state를 LiDAR frame 끝 시각까지 예측
 ├─ 각 IMU 시각의 pose를 저장 (상대 시간에 맞는 pose로 저장)
 └─ 마지막에 frame_end_time까지 한 번 더 맞춤
*/
void SlamCore::predictByImu(const MeasureGroup& meas)
{
    auto imu_sequence = prepareImuSequence(meas);
    //prepareImuSequence : 이번 프레임 예측에 쓸 IMU 목록을 만듬
    //이전 frame의 마지막 imu값인 last_imu_도 넣어서 같이 붙여줌.
    //왜냐하면 현재 프레임시작 직전부터 이어서 예측해야 하기 때문



    if(imu_sequence.size() <2) return;

    resetDeskewPoses(imu_sequence.size()+1); 
    // 이번 frame 전용 pose 기록을 비움
    //왜 + 1하지? 
    /*답변: 
        LiDAR frame begin : 10.000
            IMU timestamps:
            10.005
            10.010
            10.015
            10.020
            10.025
        LiDAR frame end   : 10.030
        이렇게 있다면, 10.025까지가 imu값이다. 이때 10.030때의 pose도 예측을 해야 하므로,,, +1을 한것이다. 
        그 예측하는 기능을 후에 predictToLidarFrameEnd() 여기서 한다.
     */


    /*두개의 */
    for(std::size_t i =0 ; i+1 < imu_sequence.size(); ++i)
    {
        predictOneImuInterval(imu_sequence[i], imu_sequence[i+1], meas);
        // imu 사이의 짧은 시간 구간 하나 잡고, acc평균, gyr평균 계산하고 나중에 deskew에 쓸 정보를 기록.
    }
    predictToLidarFrameEnd(meas);
    //LiDAR frame 끝 시각(10.030)의 pose 값을 예측하기 위한 함수
    updatePredictionTail(meas);
    //이번 예측이 끝난 뒤, 다음 frame에서 이어서쓰기위한 tail정보를 저장함. 

    

}

//이미 존재하는 frame 내의 imus값 맨 앞에 이전 frame의 마지막 imu값 넣기
//다만 기존것인 meas.imus에 넣으면 measureGroup이라는 성질이 이상해짐.. 그래서 다시 vector 만든거
std::vector<sensor_msgs::msg::Imu::SharedPtr> SlamCore::prepareImuSequence(const MeasureGroup& meas) const
{
    std::vector<sensor_msgs::msg::Imu::SharedPtr> seq;
    seq.reserve(meas.imus.size() +1) ;
    //이전 frame의 imu마지막 값을 이번frame 맨앞에 두기위해서  한칸 더 메모리확보
    
    if(last_imu_) seq.push_back(last_imu_);
    //last_imu_라고 이름이 되어 있지만, 사실 직전frame의 마지막 imu msg값임.
    //마지막값을 맨처음에 넣음

    for(const auto& imu : meas.imus) 
    {
        seq.push_back(imu);
    }
    return seq;
}
void SlamCore::resetDeskewPoses(std::size_t expected_size)
{
    deskew_poses_.clear();
    deskew_poses_.reserve(expected_size);
}

/*imu 사이의 짧은 시간 구간 하나 잡고, acc평균, gyr평균 계산하고, 나중에 deskew에 쓸 정보를 기록한다.*/
void SlamCore::predictOneImuInterval(
    const sensor_msgs::msg::Imu::SharedPtr& head,
    const sensor_msgs::msg::Imu::SharedPtr& tail,
    const MeasureGroup& meas
    )
{
    /*순서 
    1. 
    */
   const double head_time = toSec(head->header.stamp);
   const double tail_time = toSec(tail->header.stamp);

   /*이전 LiDAR frame 끝보다 더 이전 구간은 의미 없음*/
   if(tail_time <= last_lidar_end_time_) return;

   /*현재 frame 시작과 겹치는 구간이면, 시작점을 잘라서 사용*/
   const double interval_beg_time = std::max(head_time, last_lidar_end_time_);
   //왜지? 어짜피 직전 frame의 imu마지막msg 값이 seq 시작에 들어갔다. 근데 왜 이렇게?
   /*답변: 
        이전 LiDAR frame end : 10.030

        이전 frame에서 마지막으로 가지고 있던 IMU:
        last_imu_ = 10.025

        현재 frame 첫 IMU:
        meas.imus.front() = 10.035

        그럼 여기에서
        head = last_imu_ = 10.025
        tail = current first imu = 10.035
        이렇게 되어 버린다. 
        이전 프레임에서 이미 10.030까지 예측은 했지만 그 값을 meas에 넣은게 아니었다. 
        그래서! 이번 frame의 시작은 10.025가 아닌 10.03으로 하기 위해서 max를 넣는거다. 

        imu_sequence
        |--------------------------------------------------|
        10.025      10.035      10.040      10.045
        이 그림에서 중요한 것은 imu_seq안에는 imu_msg값만 있기 때문에 predict한 10.03값이 없다는 거다. 

        head = 10.025
        tail = 10.035
        이 상태에서 이미 10.030까지 처리 해서 pose를 predict한 상태이다. 그러니 적분 필요 없다. 
        따라서   max로 10.025가 아닌 10.030을 head로해서  interval을 10.030 과 10.035 사이를 적분한다. 

        시간축
        |--------------------------------------------------------------------->

        이전 frame
                                                last_imu_
                                                    |
                                                    v
            10.005   10.010   10.015   10.020   10.025            10.030(seq 없음)
                |--------|--------|--------|--------|---------------|
                                                                    ^
                                                                    |
                                                            last_lidar_end_time_
                                                            (이전 LiDAR frame end)
                                                            (이번 LiDAR frame beg)

        현재 frame
                                                                            10.035   10.040
                                                                    |--------|--------|
                                                                            ^
                                                                            |
                                                                    meas.imus.front()
   */
    const double dt = tail_time - interval_beg_time;
    // 0.005 = (10.035 - 10.030)


    if(dt <= 0.0) return;


    Eigen::Vector3d gyro_avg(
        0.5* (head->angular_velocity.x + tail->angular_velocity.x),
        0.5* (head->angular_velocity.y + tail->angular_velocity.y),
        0.5* (head->angular_velocity.z + tail->angular_velocity.z)
    );

    Eigen::Vector3d acc_avg(
        0.5* (head->linear_acceleration.x + tail->linear_acceleration.x),
        0.5* (head->linear_acceleration.y + tail->linear_acceleration.y),
        0.5* (head->linear_acceleration.z + tail->linear_acceleration.z)
    );


    //초기 평균 중력 크기로 scale 맞춤
    if(mean_acc_.norm() > 1e-6) 
        acc_avg *=(9.81 /mean_acc_.norm());
    /*
    상황1
    mean_acc_ = (0.0, 0.0, 9.70) 
    acc_avg   = (0.1, 0.2, 9.70);
                이면 멈춰있을때  가속도z는 9.81 되어야 하니까.    
    scale = 9.81 / 9.70
      ≈ 1.0113 되어서
    acc_avg = (0.101, 0.202, 9.81)
    이렇게 scale이 맞춰진다.
    */

    // TODO:
    // 1. bias 보정
    // 2. 상태 predict
    // 3. deskew용 pose 저장

    const Eigen::Vector3d gravity(0.0, 0.0, -9.81);
    // 지금은 임시로 고정시킴, 하지만 나중에는 초기화로 구한 중력방향 써야함. 
 
    // 1) interval마다 회전행렬을 구하고 현재 추정되는 회전값을 누적
    updatePredRot(gyro_avg, dt);

    // 2) world 기준 가속도
    Eigen::Vector3d acc_world = pred_rot_ * acc_avg + gravity;
    //IMU 몸체 기준 좌표계가 아닌 world기준이어야 하니까 
    // IMU 좌표계 acc ---> (현재 회전 자세 pred_rot_) --> world 좌표계 acc


    // 3) 속도, 위치 업데이트
    // 고전 등가속도 운동식 
    //      위치 p = p + v*dt+ 1/2*a*dt^2
    //      속도 v = v + a*dt
    pred_pos_ = pred_pos_ + pred_vel_ * dt + 0.5 * acc_world * dt * dt;
    pred_vel_ = pred_vel_ + acc_world * dt;



    /*save DeskewPose*/
    //여기 아직 bias보정은 안들어감. 
    DeskewPose pose;
    pose.offset_time = tail_time - meas.lidar_frame.frame_beg_time; //lidar frame 시작 기준으로 이 IMU 예측 결과가 몇 초 지점의 값인지 저장
    pose.gyro = gyro_avg;
    pose.acc = acc_avg;
    pose.rot = pred_rot_;
    pose.vel = pred_vel_;
    pose.pos = pred_pos_;

    deskew_poses_.push_back(pose);
}


void SlamCore::updatePredRot(const Eigen::Vector3d& gyro_avg, double dt)
{
   /*
    IMU propagation 순서 
        1. gyro로 회전 예측
        2. 회전값으로 acc를 world 좌표계로 변환
        3. 속도 업데이트
        4. 위치 업데이트
    
    
    */

    // 1) 회전 업데이트(imu propagation 핵심)
    // gyro_avg로 회전 예측하고 acc_avg로 속도와 위치를 예측하는 코드
    // imu gyro -> 회전 변화량 계산 -> pred_rot_ 업데이트
    Eigen::Vector3d dtheta = gyro_avg * dt;
    // dtheta  1. 어느 회전 방향으로  2. 얼마나 회전했는가  정보가 들어 있다.
    // dtheta 방향 : 회전축 방향(axis)
    // dtheta 크기(norm) = 회전각(angle)
    // 평균각속도(rad/s) * dt(s) -> interval 시간 동안 얼마나 회전했는지 안다. 

    // gyro_avg = (0, 0, 0.2) rad/s, dt = 0.005 s , 이걸 곱하면 dtheta는 (0,0,0.001), 
    //즉 이번 interval동안 z축 방향으로 0.001 rad만큼 회전했다는 것임.
    // dtheta안에는 1. 어느 축으로? 2.얼마나 회전 이런것을 담고 있다. 

    double angle = dtheta.norm();
    //dtheta는 3D벡터이다. 이것을 norm()하면 벡터의 크기가 나옴. 
    //회전의 전체 크기는 총회전각이고  angle = sqrt(0² + 0² + 0.001²) = 0.001 이렇게 된다. 
    Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
    //처음에는 회전량 변화없음 dR=I다. 그 뒤... angle이 충분히 커지면, 회전 행렬을 만든다.

    /*angle이 거의 0이면 나눔 문제 생기므로, 회전 변화 없으면 I로 유지, 0이 아니게 되면,axis, angle로 dR구한다*/
    if (angle > 1e-12) {
        Eigen::Vector3d axis = dtheta / angle;

        // axis : 회전축만 뽑힘.
        //ex) dtheta = (0, 0, 0.001) , angle = 0.001,  axis = (0, 0, 1)
        
        dR = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
        //axis방향을 회전축으로 해서, angle만큼 회전하는 3x3 회전행렬 만든다. 그리고 dR은 이번 interval동안 회전 변화량임
        //dR = 이번 0.005초 동안 IMU가 회전한 만큼의 회전행렬
        /*
        dtheta = (0, 0, 0.001)
        angle = 0.001
        axis = (0, 0, 1)
        그럼 여기서 Eigen::AngleAxisd(0.001, Eigen::Vector3d(0,0,1)) 되고, z축 기준으로 0.001 rad 회전이란 뜻되고, 
        이걸 회전행렬로 바꾸면, 
        dR =
            [ cos(0.001)  -sin(0.001)   0 ]
            [ sin(0.001)   cos(0.001)   0 ]
            [     0            0        1 ]
                 
        왜 회전행렬로 바꾸냐면?
         dtheta는 사람이 보기에는 좋은 벡터지만,  현재 자세인(pred_rot_)와 누적하려면 회전 연산이필요하다. 
         회전 연산은 단순한 덧셈이 아니어서 행렬 곱을 통해서 해야 한다. 
        */
    }

    pred_rot_ = pred_rot_ * dR;
    // pred_rot_ : 현재까지 추정되는 회전값.
    // 지금까지 자세에 이번 interval의 작은 회전을 추가한다.
}


//LiDAR frame 끝 시각(10.030)의 pose 값을 예측하기 위한 함수
/*
순서
 1. measure 가 empty이면 return
 2. 인터벌 시간  = 현frame의 마지막 시간 - imu의 back() 시간
 3. 마지막 imu값으로 gyro_avg,acc_avg 구함
 4. 초기 중력 크기로 scale 맞춤
 5. updatePredRot()
 6. IMU기준에서 world기준으로 바꾸기
 7. save deskewPose
*/
void SlamCore::predictToLidarFrameEnd(const MeasureGroup& meas) 
{
    if(meas.imus.empty()) return;

    const double imu_end_time = toSec(meas.imus.back()->header.stamp);
    const double dt = meas.frame_end_time - imu_end_time;

    if(dt <= 0.0) return;

    std::cout
        << "[predictToLidarFrameEnd] "
        << "imu_end_time=" << imu_end_time
        << ", lidar_end_time=" << meas.frame_end_time
        << ", dt=" << dt
        << std::endl;

    const auto& last_imu = meas.imus.back();

    Eigen::Vector3d gyr_last(
        last_imu->angular_velocity.x,
        last_imu->angular_velocity.y,
        last_imu->angular_velocity.z
    );

    Eigen::Vector3d acc_last(
        last_imu->linear_acceleration.x,
        last_imu->linear_acceleration.y,
        last_imu->linear_acceleration.z
    );

    if( mean_acc_.norm() > 1e-6)
        acc_last *=(9.81 /mean_acc_.norm());

        const Eigen::Vector3d gravity(0.0, 0.0, -9.81);


    /*world 기준 가속도*/
    updatePredRot(gyr_last,dt);
    Eigen::Vector3d acc_world = pred_rot_ * acc_last + gravity;

    /*속도 위치 업데이트*/
    pred_pos_ = pred_pos_ + pred_vel_ * dt + 0.5 * acc_world * dt * dt;
    pred_vel_ = pred_vel_ + acc_world * dt;
    
    /*저장*/
    DeskewPose pose;
    pose.offset_time = meas.frame_end_time - meas.lidar_frame.frame_beg_time;
    pose.gyro = gyr_last;
    pose.acc = acc_last;
    pose.rot = pred_rot_;
    pose.vel = pred_vel_;
    pose.pos = pred_pos_;

    deskew_poses_.push_back(pose);

}
// 다음 frame에서 IMU적분을 끊기지 않고 이어가게 하기 위함.
// 이것을 통해서 다음 frame의 첫 IMU와 이어서 interval 만들 수있다. 
void SlamCore::updatePredictionTail(const MeasureGroup& meas) 
{
    if(!meas.imus.empty())  
        last_imu_ = meas.imus.back();
    
    last_lidar_end_time_ = meas.frame_end_time;
}

/*
LiDAR 한 프레임 안의 각 점은 서로 달느 시각에 찍힘
어떤것은 frame 앞단, 어떤 것은 끝직전,
여기까지 pred_rot_, pred_pos_, pred_vel_,들을 통해서
deskew_pose_로 저장했고, 각 시각마다 센서가 어디 있었는지 기록해두었다. 

이제 그 기록을 써서 "각 점을 frame end 시각 기준"으로 좌표를 옮긴다.

순서: 
1.end_pose잡기
2. 각 point의 relative_time 읽기
3. 그 시간에 가장 가까운 DeskewPose찾기
4. 점 옮기기
*/
void SlamCore::undistortLidar(const MeasureGroup& meas)
{
    if(!meas.lidar_frame.cloud) return;
    if(meas.lidar_frame.cloud->points.empty()) return;
    if(deskew_poses_.empty())  return;

    auto undistorted_cloud = std::make_shared<PointCloudXYZIT>();
    undistorted_cloud->points.reserve(meas.lidar_frame.cloud->points.size());
    //최적화위해서 처음부터 메모리 공간 확보

    const auto& end_pose = deskew_poses_.back();
    //lidar frame의 end를 기준으로 pose 잡음. 

    /*point를 하나씩 처리*/
    //원본을 하나씩 보면서 각 point마다 해당 시점의 pose를 찾기
    for(const auto& src_pt : meas.lidar_frame.cloud->points) 
    {
        
        const DeskewPose* cur_pose = findDeskewPose(src_pt.relative_time);
        //현재 시간에 맞는 pose를 찾기
        //cur_pose 는 world 기준 로봇의 위치다.
        //src_pt.relative_time = 0.035 이라면, 
        /*
        deskew_poses_ 안에서 offset_time이 0.035 근처인 pose를 찾아야함.
            offset_time = 0.005
            offset_time = 0.010
            offset_time = 0.015
            ...
            offset_time = 0.100
        */
        if(!cur_pose) continue;
        Eigen::Vector3d pt_body(src_pt.x, src_pt.y, src_pt.z);
        //LiDAR가 측정한 point 좌표
        //point를 Eigen벡터로 변환
        //world 좌표기준이 아니라 LiDAR 좌표계 기준이다.

        #if 0
        Eigen::Vector3d pt_world = cur_pose->rot * pt_body + cur_pose->pos;
        //현재point가 찍힌 시점의 body 좌표계 -> world 좌표로 변환
        //수식 : pt_world = R_cur * pt_body + t_cur
        //R_cur = cur_pose->rot, 
        //pt_body : 현재 시점 LiDAR 기준 point
        //t_cur = cur_pose->pos이다. 
        //pt_world : 현재 시점 world 기준 point
        /* 
        예를 들어서, 
            cur_pose.pos = (10, 0, 0)
            cur_pose.rot = Identity
            pt_body = (2, 1, 0)
            이러면, 
            pt_world= I * (2, 1, 0) + (10, 0, 0)
                    = (12, 1, 0)
            로봇이 world에서 x=10 위치에 있었고, 
            LiDAR기준으로 앞쪽 2m, 옆1m에 점이보였다. 
            그러면 world 기준 점 위치는(12,1,0)이다.
            아... cur_pose가 world기준의 LiDAR위치구나. 
        */

        Eigen::Vector3d pt_end = end_pose.rot.transpose() * (pt_world - end_pose.pos);
        //world에 올린 point를 다시 LiDAR frame end 시점의 body좌표계로 내린다. 
        //수식: pt_end = R_end^T * (p_world - t_end)
        //왜 transpose()하냐면, 회전행렬의 inverse가 transpose라서
        //즉 R^-1 = R^T
        //말로 풀면, world기준 point에서 end_pose 위치를 빼고, 
        //end_pose 회전의 inverse를 곱해서, frame end시점의 LiDAR/body 좌표계로 변환함
        // end_pose.rot.transpose() : frame end시점의 LiDAR/body 좌표계
        //이 결과 pt_end가 바로 deskew된 point다.
        //point가 찍힌 시점 cur_pose 기준 point ->world로 변환 -> frame_end pose 기준으로 point로 변환
        // end_pose.pos 는 LiDAR frame end시점의 world기준 상 로봇 위치다
        /*
            end_pose.pos = (10, 0, 0)
            end_pose.rot = Identity

            pt_world = (12, 3, 0)
            이 point를 end frame 기준으로 보면, 
            로봇이 x=10에 있고, point가 x=12, y=3에 있으니, 
            로봇 기준 point = (2,3,0)이다. 이래서 빼는거다. 

            end_pose.rot.transpose()는 
            pt_world - end_pose.pos까지의 좌표축이 world 기준이라서
            end시점의 body/LiDAR 좌표계 축 기준으로 표현하고 싶기 때문이다.
                body 좌표 → world 좌표 : R_end
                world 좌표 → body 좌표 : R_end^-1 

            예시 
                cur_pose.pos = (0, 0, 0)
                cur_pose.rot = I

                end_pose.pos = (1, 0, 0)
                end_pose.rot = I

                pt_body = (5, 0, 0)
                
                pt_world = I * (5,0,0) + (0,0,0)
                        = (5,0,0)
                현재 시점에서 LiDAR가 point를 5m 앞에서 봄...
                그런데 frame end시점에는 로봇이 x=1만큼 앞으로 이동함.
                이때 frame end 기준으로 이 point는 몇  m 앞에 이을까?
                pt_end = I^T * ((5,0,0) - (1,0,0))
                    = (4,0,0)
                즉 원래 5m 앞에 있던 점이 로봇이 1m 앞으로 이동한 뒤에는 4m앞에 있는 것처럼 보임.
                이게 motion compensation임.

        */
        #elif 1
        Eigen::Vector3d pt_end =
            end_pose.rot.transpose() * (cur_pose->rot * pt_body);
        #endif 

        /*보정된 point 저장*/
        //point 나머지정보는 그대로 유지, relative_time, tag ...
        PointType dst_pt = src_pt;
        dst_pt.x = static_cast<float>(pt_end.x());
        dst_pt.y = static_cast<float>(pt_end.y());
        dst_pt.z = static_cast<float>(pt_end.z());

        undistorted_cloud->points.push_back(dst_pt);


    }

    /*cloud metadata 갱신*/
    undistorted_cloud->width = static_cast<std::uint32_t>(undistorted_cloud->points.size());
    //point개수
    undistorted_cloud->height = 1; // unorgaized point cloud
    undistorted_cloud->is_dense = false;

    //todo : publisher로 보내기.
    undistorted_cloud_ = undistorted_cloud;
    
    std::cout
    << "[undistortLidar] src=" << meas.lidar_frame.cloud->points.size()
    << ", dst=" << undistorted_cloud_->points.size()
    << std::endl;
}

/*point가 찍힌 시간 point_time보다 작거나 같은 가장 가까운 pose를 찾기
    deskew_poses_ offset_time:
    0.005
    0.010
    0.015
    0.020
    0.025
    이때 point시간인 point_time = 0.017이라면, 
    0.015를 취한다. 

*/
const  DeskewPose* SlamCore::findDeskewPose(double point_time)  const
{
    if(deskew_poses_.empty()) return nullptr;
    const DeskewPose* result = &deskew_poses_.front();

    for(const auto& pose : deskew_poses_) 
    {
        if(pose.offset_time > point_time) break;
        result  = &pose;
    }
    return result; 
}



// void SlamCore::pushLidarMsg(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
void SlamCore::pushLidarFrame(const LidarFrame& lidar_frame )
{
    std::cout << "pushLidarMsg" << std::endl;
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    /*bag파일문제나 센서,드라이버 문제로 시간 역전 현상을 방지하기 위해서 last_timestamp_lidar_로 항상 관리함*/
    if(lidar_frame.frame_beg_time < last_timestamp_lidar_)
    {
        std::cout << "[SlamCore] lidar loop back, clear lidar buffer" << std::endl;
        lidar_frame_buffer_.clear();
    }
    last_timestamp_lidar_ = lidar_frame.frame_beg_time;
    
    /*둘을 항상 짝맞춰 줘야 하기 때문에, 메시지버터와 시간 버퍼를 나눠서 관리*/
    // lidar_buffer_.push_back(msg);
    // lidar_time_buffer_.push_back(lidar_frame.frame_beg_time);
    lidar_frame_buffer_.push_back(lidar_frame);
}


void SlamCore::pushImuMsg(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::cout << "pushImuMsg" << std::endl;
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    const double timestamp = toSec(msg->header.stamp);

    /*시간 역순 방지하기 위해서 clear함*/
    if(timestamp < last_timestamp_imu_) 
    {
        std::cout << "[SlamCore] imu loop back, clear imu buffer" << std::endl;
        imu_buffer_.clear();
    }
    last_timestamp_imu_ = timestamp;
    imu_buffer_.push_back(msg);

}

void SlamCore::spinFrontendOnce()
{
    std::cout << "[SlamCore] spinFrontendOnce" << std::endl;

    MeasureGroup meas;

    if (!syncMeasureGroup(meas)) {
        std::cout << "[SlamCore] syncMeasureGroup false" << std::endl;
        return;
    }

    std::cout << "[debug] sync success" << std::endl;

    if (!meas.lidar_frame.cloud) {
        std::cout << "[debug] meas.lidar_frame.cloud is null" << std::endl;
        return;
    }

    std::cout << "[debug] cloud ok" << std::endl;

    std::cout
        << "[Measure] "
        << "lidar pts = " << meas.lidar_frame.cloud->points.size()
        << ", imu count = " << meas.imus.size()
        << ", lidar beg = " << meas.lidar_frame.frame_beg_time
        << ", lidar end = " << meas.frame_end_time
        << std::endl;

    processMeasure(meas);   
}

bool SlamCore::saveMap()
{
    std::cout << "saveMap" << std::endl;
    return true;
}

PointCloudXYZITConstPtr SlamCore::getDebugPreprocessCloud() const
{
    return debug_preprocess_cloud_;
}

PointCloudXYZITConstPtr SlamCore::getUndistortedCloud() const
{
    return undistorted_cloud_;
}

double SlamCore::getLastProcessedFrameTime() const
{
    return last_processed_frame_time_;
}