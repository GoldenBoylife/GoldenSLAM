#include "include/core/api/esekfom_api.hpp"


EsekfomApi::EsekfomApi()
{

}

void EsekfomApi::setParams(const SlamParams& params)
{
    has_params_ = true;
}

// void EsekfomApi::pushImu(const ImuData& imu)
// {
//     imu_count_++;

//     if(imu_count_ % 100 == 0) 
//         std::cout << "[EsekfomApi]  imu_count = " << imu_count_ << std::endl;
// }

void EsekfomApi::propagateOnce()
{
    if( !has_params_) return;
}

void EsekfomApi::updateOnce()
{
    if(!has_params_) return;
}