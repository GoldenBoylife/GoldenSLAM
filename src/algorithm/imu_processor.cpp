#include "algorithm/imu_processor.hpp"
#include <omp.h>
#include <cmath>
#include <third_party/so3_math.h>

#define MAX_INI_COUNT (10)

static bool time_list(PointType& x, PointType& y)
{
    return x.curvature < y.curvature;
}

// ─────────────────────────────────────────────────────────────────────────────

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
    init_iter_num  = 1;
    Q              = process_noise_cov();
    cov_acc        = V3D(0.1, 0.1, 0.1);
    cov_gyr        = V3D(0.1, 0.1, 0.1);
    cov_bias_gyr   = V3D(0.0001, 0.0001, 0.0001);
    cov_bias_acc   = V3D(0.0001, 0.0001, 0.0001);
    mean_acc       = V3D(0, 0, -1.0);
    mean_gyr       = V3D(0, 0, 0);
    angvel_last    = Zero3d;
    Lidar_T_wrt_IMU = Zero3d;
    Lidar_R_wrt_IMU = Eye3d;
    last_imu_      = ImuData{};
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset()
{
    mean_acc         = V3D(0, 0, -1.0);
    mean_gyr         = V3D(0, 0, 0);
    angvel_last      = Zero3d;
    imu_need_init_   = true;
    start_timestamp_ = -1;
    init_iter_num    = 1;
    v_imu_.clear();
    IMUpose.clear();
    last_imu_        = ImuData{};
    cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::Reset(double start_timestamp, const ImuData& lastimu)
{
    Reset();
    start_timestamp_ = start_timestamp;
    last_imu_        = lastimu;
}

void ImuProcess::set_extrinsic(const MD(4,4)& T)
{
    Lidar_T_wrt_IMU = T.block<3,1>(0,3);
    Lidar_R_wrt_IMU = T.block<3,3>(0,0);
}

void ImuProcess::set_extrinsic(const V3D& transl)
{
    Lidar_T_wrt_IMU = transl;
    Lidar_R_wrt_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const V3D& transl, const M3D& rot)
{
    Lidar_T_wrt_IMU = transl;
    Lidar_R_wrt_IMU = rot;
}

void ImuProcess::set_gyr_cov(const V3D& scaler)   { cov_gyr_scale = scaler; }
void ImuProcess::set_acc_cov(const V3D& scaler)   { cov_acc_scale = scaler; }
void ImuProcess::set_gyr_bias_cov(const V3D& b_g) { cov_bias_gyr  = b_g; }
void ImuProcess::set_acc_bias_cov(const V3D& b_a) { cov_bias_acc  = b_a; }

// ─── IMU_init ─────────────────────────────────────────────────────────────────
void ImuProcess::IMU_init(const MeasureGroup& meas, IEkf& kf, int& N)
{
    V3D cur_acc, cur_gyr;

    if (b_first_frame_)
    {
        Reset();
        N = 1;
        b_first_frame_ = false;
        mean_acc = meas.imu.front().linear_acceleration;
        mean_gyr = meas.imu.front().angular_velocity;
        first_lidar_time = meas.lidar_beg_time;
    }

    for (const auto& imu : meas.imu)
    {
        cur_acc = imu.linear_acceleration;
        cur_gyr = imu.angular_velocity;

        mean_acc += (cur_acc - mean_acc) / N;
        mean_gyr += (cur_gyr - mean_gyr) / N;

        cov_acc = cov_acc * (N-1.0)/N + (cur_acc-mean_acc).cwiseProduct(cur_acc-mean_acc) * (N-1.0)/(N*N);
        cov_gyr = cov_gyr * (N-1.0)/N + (cur_gyr-mean_gyr).cwiseProduct(cur_gyr-mean_gyr) * (N-1.0)/(N*N);
        N++;
    }

    state_ikfom init_state = kf.get_x();
    init_state.grav = S2(-mean_acc / mean_acc.norm() * G_m_s2);
    init_state.bg   = mean_gyr;
    init_state.offset_T_L_I = Lidar_T_wrt_IMU;
    init_state.offset_R_L_I = Lidar_R_wrt_IMU;
    kf.change_x(init_state);

    IEkf::CovType init_P = kf.get_P();
    init_P.setIdentity();
    init_P(6,6)   = init_P(7,7)   = init_P(8,8)   = 0.00001;
    init_P(9,9)   = init_P(10,10) = init_P(11,11) = 0.00001;
    init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;
    init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;
    init_P(21,21) = init_P(22,22) = 0.00001;
    kf.change_P(init_P);
    last_imu_ = meas.imu.back();
}

// ─── UndistortPcl ─────────────────────────────────────────────────────────────
void ImuProcess::UndistortPcl(const MeasureGroup& meas, IEkf& kf, PointCloudXYZI& pcl_out)
{
    auto v_imu = meas.imu;
    v_imu.push_front(last_imu_);
    const double imu_beg_time = v_imu.front().timestamp;
    const double imu_end_time = v_imu.back().timestamp;
    const double pcl_beg_time = meas.lidar_beg_time;
    const double pcl_end_time = meas.lidar_end_time;
    (void)imu_beg_time;

    pcl_out = *(meas.lidar);
    sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);

    state_ikfom imu_state = kf.get_x();
    IMUpose.clear();
    IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last,
                                 imu_state.vel, imu_state.pos,
                                 imu_state.rot.toRotationMatrix()));

    V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
    M3D R_imu;
    double dt = 0;
    input_ikfom in;

    for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)
    {
        const ImuData& head = *it_imu;
        const ImuData& tail = *(it_imu + 1);

        double tail_stamp = tail.timestamp;
        double head_stamp = head.timestamp;

        if (tail_stamp < last_lidar_end_time_) continue;

        angvel_avr = 0.5 * (head.angular_velocity  + tail.angular_velocity);
        acc_avr    = 0.5 * (head.linear_acceleration + tail.linear_acceleration);

        acc_avr = acc_avr * G_m_s2 / mean_acc.norm();

        if (head_stamp < last_lidar_end_time_)
            dt = tail_stamp - last_lidar_end_time_;
        else
            dt = tail_stamp - head_stamp;

        in.acc  = acc_avr;
        in.gyro = angvel_avr;
        Q.block<3,3>(0,0).diagonal() = cov_gyr;
        Q.block<3,3>(3,3).diagonal() = cov_acc;
        Q.block<3,3>(6,6).diagonal() = cov_bias_gyr;
        Q.block<3,3>(9,9).diagonal() = cov_bias_acc;
        kf.predict(dt, Q, in);

        imu_state   = kf.get_x();
        angvel_last = angvel_avr - imu_state.bg;
        acc_s_last  = imu_state.rot * (acc_avr - imu_state.ba);
        for (int i = 0; i < 3; i++) acc_s_last[i] += imu_state.grav[i];

        double offs_t = tail_stamp - pcl_beg_time;
        IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last,
                                     imu_state.vel, imu_state.pos,
                                     imu_state.rot.toRotationMatrix()));
    }

    double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
    dt = note * (pcl_end_time - imu_end_time);
    kf.predict(dt, Q, in);

    imu_state = kf.get_x();
    last_imu_ = meas.imu.back();
    last_lidar_end_time_ = pcl_end_time;

    if (pcl_out.points.begin() == pcl_out.points.end()) return;

    auto it_pcl = pcl_out.points.end() - 1;
    for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
    {
        auto head = it_kp - 1;
        auto tail = it_kp;
        R_imu   << MAT_FROM_ARRAY(head->rot);
        vel_imu << VEC_FROM_ARRAY(head->vel);
        pos_imu << VEC_FROM_ARRAY(head->pos);
        acc_imu << VEC_FROM_ARRAY(tail->acc);
        angvel_avr << VEC_FROM_ARRAY(tail->gyr);

        for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--)
        {
            dt = it_pcl->curvature / double(1000) - head->offset_time;

            M3D R_i(R_imu * Exp(angvel_avr, dt));
            V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
            V3D T_ei(pos_imu + vel_imu*dt + 0.5*acc_imu*dt*dt - imu_state.pos);
            V3D P_compensate = imu_state.offset_R_L_I.conjugate() *
                               (imu_state.rot.conjugate() *
                                (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei)
                                - imu_state.offset_T_L_I);

            it_pcl->x = P_compensate(0);
            it_pcl->y = P_compensate(1);
            it_pcl->z = P_compensate(2);

            if (it_pcl == pcl_out.points.begin()) break;
        }
    }
}

// ─── Process ──────────────────────────────────────────────────────────────────
void ImuProcess::Process(const MeasureGroup& meas, IEkf& kf, PointCloudXYZI::Ptr cur_pcl_un_)
{
    if (meas.imu.empty()) return;
    assert(meas.lidar != nullptr);

    if (imu_need_init_)
    {
        IMU_init(meas, kf, init_iter_num);
        imu_need_init_ = true;
        last_imu_ = meas.imu.back();

        if (init_iter_num > MAX_INI_COUNT)
        {
            cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
            imu_need_init_ = false;
            cov_acc = cov_acc_scale;
            cov_gyr = cov_gyr_scale;
            std::cout << "IMU Initial Done\n";
            fout_imu.open(DEBUG_FILE_DIR("imu.txt"), std::ios::out);
        }
        return;
    }

    UndistortPcl(meas, kf, *cur_pcl_un_);
}
