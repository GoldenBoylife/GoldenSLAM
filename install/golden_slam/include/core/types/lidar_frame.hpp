#pragma once

#include "core/types/pcl_types.hpp"

struct LidarFrame
{
    double frame_end_time =0.0; 
    double frame_beg_time =0.0; 
    CloudTPtr cloud = nullptr; 

};

struct LidarConvertResult
{
    CloudTPtr cloud = nullptr;
    double max_relative_time = 0.0;
};




