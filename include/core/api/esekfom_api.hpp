#pragma once

#include "core/types/slam_types.hpp"

class EsekfomApi
{
public:
    EsekfomApi();
    void setParams(const SlamParams& params);
    // void pushImu(const ImuData& imu);
    void propagateOnce();
    void updateOnce();

private: 
    bool has_params_ = false;
    int imu_count_ =0;

};