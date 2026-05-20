#pragma once

#include "core/types/pcl_types.hpp"
#include "core/types/state.hpp"

struct FrontendSnapshot
{
    bool valid = false;
    double stamp = 0.0;
    State predicted_state;

    CloudTConstPtr cloud_world_predicted;
    CloudTConstPtr cloud_deskewed;
    CloudTConstPtr cloud_world_deskewed;

    CloudTConstPtr debug_map_predicted;
    CloudTConstPtr debug_map_deskewed;
    
    /*나중에 undistort와 iEKF 생기면 추가*/
};