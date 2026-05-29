#include "algorithm/iekf.hpp"
#include <algorithm>

IEkf::IEkf()
{
    std::fill(epsi_, epsi_ + 23, 0.001);
}

void IEkf::init(int max_iterations, HModel h_model)
{
    kf_.init_dyn_share(get_f, df_dx, df_dw, h_model, max_iterations, epsi_);
    //여기서 h_model이 함수 포인터이고, 여기서 함수 포인터가 등록되었다. 
}

void IEkf::predict(double dt, Eigen::Matrix<double, 12, 12>& Q, input_ikfom& in)
{
    kf_.predict(dt, Q, in);
}

void IEkf::update(double laser_point_cov, double& solve_time)
{
    kf_.update_iterated_dyn_share_modified(laser_point_cov, solve_time);
    //이미 slamCore 초기화 때 
    //h_share_model_static함수를 ekfom 내부에 등록 시켜 두어서 콜백 함수화되었고, 
    //여기서 이 update가 실행될 때 h_share_model_static함수가 콜백을 반복 호출함. 
    //eskfom이 "measurement 필요하네"라고 등록된 함수 호출
}

state_ikfom IEkf::get_x()
{
    return kf_.get_x();
}

IEkf::CovType IEkf::get_P()
{
    return kf_.get_P();
}

void IEkf::change_x(state_ikfom x)
{
    kf_.change_x(x);
}

void IEkf::change_P(CovType P)
{
    kf_.change_P(P);
}
