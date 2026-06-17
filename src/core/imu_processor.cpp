#include "include/core/imu_processor.hpp"

ImuProcessor::ImuProcessor()
{
    b_first_frame_ = true;
    imu_need_init_ = true;

    init_iter_num_ =1;
    first_lidar_time_ = 0.0;
    last_lidar_end_time_ = 0.0;

    // mean_acc_ = Eigen::Vector3d(0.0, 0.0, -1.0);
    mean_acc_ = V3D(0.0, 0.0, -1.0);
    mean_gyr_ = V3D::Zero();

    cov_acc_ = V3D(0.1,0.1,0.1);
    cov_gyr_ = V3D(0.0,0.1,0.1);

    cov_bias_gyr_= V3D(0.0001, 0.0001, 0.0001);
    cov_bias_acc_ = V3D(0.0001, 0.0001, 0.0001);

    cov_acc_scale_ = cov_acc_;
    cov_gyr_scale_ = cov_gyr_;

    Q_=process_noise_cov();
}

ImuProcessor::~ImuProcessor()
{
}

void ImuProcessor::reset()
{
    b_first_frame_ = true;
    imu_need_init_ = true;

    init_iter_num_ =  1;
    
    first_lidar_time_ = 0.0;
    last_lidar_end_time_ = 0.0;

    mean_acc_ = V3D(0.0, 0.0, -1.0);
    mean_gyr_ = V3D::Zero();

    cov_acc_ = V3D(0.1, 0.1, 0.1);
    cov_gyr_ = V3D(0.1, 0.1, 0.1);
    cov_bias_gyr_ = V3D(0.0001, 0.0001, 0.0001);
    cov_bias_acc_ = V3D(.0001, 0.0001, 0.0001);

    Q_= process_noise_cov();
    last_imu_ = ImuData{};

}

void ImuProcessor::setFirstLidarTime(double first_lidar_time)
{
    first_lidar_time_ = first_lidar_time;
}

/*
    - 첫 프레임이면 reset
    - mean_acc /mean_gyr 초기값 설정
    - 여러 IMU로 평균 acc /gyro 계산
    - cov_acc / cov_gyr 갱신
    - EsekfomApi 에 초기화 요청
*/
void ImuProcessor::imuInit(const MeasureGroup& meas, EsekfomApi& esekfom_api)
{
    if(meas.imus.empty())   return;

    if(b_first_frame_)
    {
        reset();

        b_first_frame_ = false;
        init_iter_num_ = 1;

        mean_acc_ = meas.imus.front().linear_acc;
        mean_gyr_ = meas.imus.front().angular_vel;
        first_lidar_time_ = meas.lidar_frame.frame_beg_time;
    }

    for(const auto& imu: meas.imus) 
    {
        const double n = init_iter_num_;
                //init_iter_num_ 은 imu초기화 때 쓰는  갯수
                //초기화에 frame 5개쓰이면,  20개*5frame으로 총 100개정도됨


        const V3D cur_acc = imu.linear_acc;
        const V3D cur_gyr = imu.angular_vel;

        
        mean_acc_ = mean_acc_ + (cur_acc - mean_acc_) / n;
        mean_gyr_ = mean_gyr_ + (cur_gyr - mean_gyr_) / n;

        /*초기 IMU 가속도 값들의 분산값을 온라인으로 업데이트*/
        //기존 cov_acc_에 이번 흔들림 값을 조금 반영한다.
        const V3D acc_diff = cur_acc - mean_acc_;
        const V3D acc_diff_sq = acc_diff.cwiseProduct(acc_diff);

        cov_acc_ =  cov_acc_ * (n -  1.0) / n 
                    + acc_diff_sq *(n-1.0) / (n*n);
        //n=1, 첫번째 값으로는 흔들림 알 수 없으니 cov 는 0이다.
        // 새cov = 예전 흔들림 점수 조금 반영 + 이번 값이 평균에서 벗어난 정도를 조금 반영 
        //cov_acc_ * (n -  1.0) / n  : 기존 cov를  반영하는 비율 정함. 
        //기존 cov에서 imu값이 9개째이고, 새로운 IMU값 들어오면, 
        //이전 cov는 90%만 반영, 이번것은 10% 반영

        //+ acc_diff_sq *(n-1.0) / (n*n); : 

        const V3D gyr_diff = cur_gyr - mean_gyr_;
        const V3D gyr_diff_sq = gyr_diff.cwiseProduct(gyr_diff);

        cov_gyr_ = cov_gyr_ * ( n - 1.0) /n 
                            + gyr_diff_sq * ( n - 1.0) / (n * n);

        

       init_iter_num_++; 
    }

    /*
    dev4에서는 IEkf 직접 만지지 않음
    평균 acc/gyr 계산 결과를 EsekfomApi에 넘기고, 
    그 클래 내부에서 sta`te_ikfom gravity, bg, covariance를 설정한다.
    
    */
   esekfom_api.initFromImuMean(mean_acc_, mean_gyr_);
   //초기화 요청
}

   
/*정지 상태로, IMU초기값 얻기*/
void ImuProcessor::process(const MeasureGroup& meas, EsekfomApi& esekfom_api)
{
    if(meas.imus.empty()) 
    {
        return;
    }

    /*정지상태인 imu를 초기값으로 잡기*/
    if(imu_need_init_)
    {
        imuInit(meas, esekfom_api);
        last_imu_ = meas.imus.back();

        last_lidar_end_time_ = meas.lidar_frame.frame_end_time;
        //초기화 중에도 LiDAR frame end 경계를 갱신해 둔다.


        if(init_iter_num_ > MAX_INIT_COUNT) 
        {
            cov_acc_ *= std::pow(G_M_S2 /mean_acc_.norm(), 2.0);
            

            imu_need_init_ = false;

            cov_acc_ = cov_acc_scale_;
            cov_gyr_ = cov_gyr_scale_;

            std::cout << "[ImuProcessor] IMU Initial Done"
                      << " mean_acc=" << mean_acc_.transpose()
                      << " mean_gyr=" << mean_gyr_.transpose()
                      << std::endl;

        }

        return;
    }
    /*초기화 마무리되면,  propagate로 IMU적분*/
    propagateImu(meas, esekfom_api);
}


/*
    - 이번 LiDAR frame에 대응되는 IMU들을 시간 순서대로 적분
    - EKF state를 LiDAR frame 끝 시간 예측
    - 중간 pose history를 저장함 
*/
void ImuProcessor::propagateImu(const MeasureGroup& meas, EsekfomApi& esekfom_api)
{
    if(meas.imus.empty()) return;


    std::deque<ImuData> cur_imu = meas.imus;

    /*
        head, tail stamp으로 dt를 구해려면, 
        이전 IMU를 고려해서 계산해야 하므로 
        이전 마지막 IMU값을 1개 앞으로 넣는다.
    */
    if(last_imu_.timestamp > 0.0)
    {
        cur_imu.push_front(last_imu_);
        //measure_group에는 last_imu는 안 넣고 dt구하기위해서 넣는다. 
    }

    /*IMU적분은 최소 2개의 imu값이 필요함, 1개라면,*/
    if(cur_imu.size() < 2) 
    {
        last_imu_ = meas.imus.back(); //
        //실제 IMU적분을 위해서 쓰임. Measuregroup에는 안넣는다.
        //이제 두번째 frame부터는 last_imu가 존재하니까 dt위해서

        last_lidar_end_time_ = meas.lidar_frame.frame_end_time;
        //ori처럼 이전 frame의 마지막 IMU를 앞에 붙인다.
        return;
    }

    int predict_count = 0;
    double sum_dt = 0.0;
    double max_dt = 0.0;

    const double lidar_beg_time = meas.lidar_frame.frame_beg_time;
    const double lidar_end_time = meas.lidar_frame.frame_end_time;
    const double lidar_dt = lidar_end_time - lidar_beg_time;


    /*
        EKF state가 어느 시간까지 predict 되었는지 추정함. 
        처음에는 이전 LiDAR frame end까지 예측되어 있다고 본다. 
        for문 안에서 tail.timestamp까지 predict할때마다 갱신한다.
        for문이 끝난 뒤, 이 값이 lidar_end_time보다 작으면  
        남은 시간 만큼 마지막 predict를 한번 더 한다.
        
    */
    double last_ekf_time = last_lidar_end_time_;
    //last_ekf_time  :  ekf가 진행된 시간된 범위.(100ms)
    //이전 lidar의 끝시간까지 ekf했을거니까. 
    

    /*2개의 imu값으로 dt계산*/
    for(std::size_t i =0; i+1 < cur_imu.size(); i++) 
    {
        const ImuData& head = cur_imu[i];
        const ImuData& tail = cur_imu[i+1];

        /*tail_stamp가 더 작다면, 이미 이전 frame에서 propagate된거니까. skip*/
        //imu tail의 시간이 이전 라이다 끝 시간보다 빠르면, 이전 프레임꺼니까 무시
        if(tail.timestamp < last_lidar_end_time_)
        {
            continue;
        }

        double dt = 0.0;

        /*        
        시작 경계 처리
            형태 : head < last_lidar_end_time  < tail
            head_stamp가 이전 프레임의 imu이라면. dt구할때 앞쪽은 last_lidar_end_time이다.

        일반 적분 
            형태: last_lidar_end_time < head < tail < cur_lidar_end_time    
            경계가 아닌 일반적인 IMU 간격은 tail - head다.
            

        */
       //imu의 head가 
        if(head.timestamp < last_lidar_end_time_)
        {
            dt = tail.timestamp - last_lidar_end_time_;
        }
        /*대다수 여기다*/
        else
        {
            dt = tail.timestamp - head.timestamp;
        }

        /*역행은 skip*/
        if(dt <= 0.0)
        {
            continue;
        }

        /*
            200Hz IMU라면 dt는 보통 0.005근처여야 한다. 0.02이상이면 timestamp gap 의심
        */
        if (dt > 0.02)
        {
            std::cout << "[ImuProcessor::propagateImu][WARN large dt] "
                << " dt=" << dt
                << " head_t=" << head.timestamp
                << " tail_t=" << tail.timestamp
                << " last_lidar_end_time=" << last_lidar_end_time_
                << " lidar_beg=" << meas.lidar_frame.frame_beg_time
                << " lidar_end=" << meas.lidar_frame.frame_end_time
                << " imu_count=" << meas.imus.size()
                << std::endl;
        }

        const V3D gyr_avg =  0.5 * (head.angular_vel + tail.angular_vel);
        V3D acc_avg = 0.5 * (head.linear_acc + tail.linear_acc);


        if(mean_acc_.norm() > 1e-6) 
        {
            acc_avg = acc_avg * ( G_M_S2 / mean_acc_.norm());
            // ( G_M_S2 / mean_acc_.norm()) : 보정 계수
            // 가속도 값 크기를 중력가속도 9.81 기준으로 스케일 보정
        }

        esekfom_api.predictImu(dt, acc_avg, gyr_avg, Q_);
        //EKF prediction step
        //현재 state로 다음 시점의 state를 예측
        //IMU에는 pose값이 없다. predictImu로 IMU의 acc,vel로 pose를 유추해본다.

        savePoseHistory(tail.timestamp - meas.lidar_frame.frame_beg_time,esekfom_api);
        //deskew를 위한 pose history
        //beg_time 쓰는 이유: relative_time이 저장되기 때문에, 시작 시간 알아야함.
        //현재 EKF state꺼내서 relative time에 해당하는 pose르 찾고 그 기준으로 point 보정

        ++predict_count;
        sum_dt += dt;


        if(dt > max_dt) 
        {
            max_dt = dt;
        }

        last_ekf_time = tail.timestamp;
        //방금 tail.timestamp까지 predict했으므로, EFK가 도달할 시간을 갱신한다.
    } //for문

    /*
        끝 경계 처리
            lidar_end_time을 넘는 IMU를 measuregroup에 넣지 않는다. 
            그래서 마지막 IMU가 198ms이고,  LiDAR end가 200ms이고, 그 다음 IMU가 203ms라서 이번 measuregroup에 포함되지 않아 있다면, 
            198ms ~200ms를 다시 dt로해서 predict해서  EKF state를 LiDAR frame end 시점에 맞춘다.
    */
   //predict_count는 적분을 하나도 못했는데, 끝보정하면 안되니까. 
    if(predict_count > 0 && last_ekf_time < lidar_end_time)
    {
        const double dt_end = lidar_end_time - last_ekf_time;

        if(dt_end > 0.0) 
        {
            if(dt_end > 0.02)
            {
                std::cout << "[ImuProcessor::propagateImu][WARN large end dt] "
                          << " dt_end=" << dt_end
                          << " last_ekf_time=" << last_ekf_time
                          << " lidar_end=" << lidar_end_time
                          << " imu_count=" << meas.imus.size()
                          << std::endl;
            }
        
            /*
                마지막 IMU 측정값을 이용해서 lidar_end_time까지 extrapolate한다.

                여기서 next IMU, 즉 lidar_end_time을 넘는 IMU를 쓰는 것이 아니다. 
                현재 MeasureGruop안에 있는 마지막 IMU값을 사용한다.
            */
            const  ImuData& last_imu_for_end = cur_imu.back();

            V3D gyr_end = last_imu_for_end.angular_vel;
            V3D acc_end = last_imu_for_end.linear_acc;

            if(mean_acc_.norm() > 1e-6) 
            {
                acc_end = acc_end * (G_M_S2 /mean_acc_.norm());
                // ( G_M_S2 / mean_acc_.norm()) : 보정 계수
                // 가속도 값 크기를 중력가속도 9.81 기준으로 스케일 보정
            }

            esekfom_api.predictImu(dt_end, acc_end, gyr_end, Q_);
            //198ms~200ms 사이의 state를 추정하는 거군요?

            savePoseHistory(lidar_end_time - lidar_beg_time,esekfom_api);
            //lidar frame 끝 시점의 pose도 history에 저장한다. deskew할때 frame끝 pose가 필요할 수 있다. 
            ++predict_count;
            sum_dt += dt_end;

            if(dt_end > max_dt)
            {
                max_dt = dt_end;
            }

            last_ekf_time = lidar_end_time;
        }
    }

    const state_ikfom state = esekfom_api.getState();

    /*frame 단위 요약 로그. 이걸 봐야 IMU propagation이 한 LiDAR frame보다 얼마나 적분되는지 판단 가능ㄴ*/
    std::cout << "[ImuProcessor::propagateImu][frame summary] "
            << " imu_count=" << meas.imus.size()
            << " cur_imu_count=" << cur_imu.size()
            << " predict_count=" << predict_count
            << " sum_dt=" << sum_dt
            << " max_dt=" << max_dt
            << " lidar_dt=" << lidar_dt
            << " last_lidar_end_time=" << last_lidar_end_time_
            << " lidar_beg=" << lidar_beg_time
            << " lidar_end=" << lidar_end_time
            << " pos=" << state.pos.transpose()
            << " vel=" << state.vel.transpose()
            << std::endl;

    last_imu_ = meas.imus.back();
    last_lidar_end_time_ = meas.lidar_frame.frame_end_time;

    // for(std::size_t i = 1; i < meas.imus.size(); ++i) 
    // {
    //     const ImuData& prev_imu = meas.imus[i-1];
    //     const ImuData& curr_imu = meas.imus[i];

    //     const double dt = curr_imu.timestamp - prev_imu.timestamp;

    //     if(dt <= 0.0)   continue;

    //     eskfom_api.predictImu(dt,curr_imu);
   

   

}

void ImuProcessor::savePoseHistory(double offset_time, const EsekfomApi& esekfom_api)
{
    //TODO: esekfom_api에서 현재 state를 읽어서 pose history로 저장
}

void ImuProcessor::undistort(const MeasureGroup& meas, EsekfomApi& esekfom_api)
{
    //TODO : pose history 사용해서 LiDAR point cloud를 distort한다.

}
