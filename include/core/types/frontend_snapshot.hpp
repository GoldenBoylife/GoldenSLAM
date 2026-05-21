#pragma once

#include "core/types/pcl_types.hpp"
#include "core/types/state.hpp"

struct FrontendSnapshot
{
    bool valid = false;
    double stamp = 0.0;
    State state;

    CloudTPtr map_cloud;
    
};