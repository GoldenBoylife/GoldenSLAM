#pragma once

#include "core/types/pcl_types.hpp"
#include "core/types/state.hpp"

struct FrontendSnapshot
{
    bool valid = false;
    double stamp = 0.0;
    State predicted_state;

    CloudTPtr cloud_world_predicted;
    CloudTPtr debug_map_predicted;
    /*나중에 undistort와 iEKF 생기면 추가*/
};