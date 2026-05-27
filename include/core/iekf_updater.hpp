#pragma once

#include <vector>
#include <Eigen/Dense>

#include "core/types/state.hpp"
#include "core/plane_estimator.hpp"

struct IekfUpdateResult
{
    bool updated = false;
    std::size_t residual_count =0;

    double mean_abs_residual = 0.0;
    double max_abs_residual = 0.0;

    /*correct 정보 */
    Eigen::Matrix<double,6,1> dx = Eigen::Matrix<double,6,1>::Zero();

    double dx_rot_norm = 0.0;
    double dx_pos_norm = 0.0;


    bool shadow_checked = false;
    bool shadow_valid_kept = false;
    bool shadow_mean_improved = false;
    bool shadow_mean_improved_enough = false;

    std::size_t shadow_before_valid = 0;
    std::size_t shadow_after_valid = 0;

    double shadow_before_mean_abs = 0.0;
    double shadow_after_mean_abs = 0.0;
    double shadow_before_max_abs = 0.0;
    double shadow_after_max_abs = 0.0;

};

class IekfUpdater
{
public:
    IekfUpdateResult update(
        const State& predicted_state,
        const std::vector<ResidualCandidate>& residuals,
        State& corrected_state);


private:
    bool buildPoseJacobian(
        const State& state,
        const ResidualCandidate& residual, 
        Eigen::Matrix<double,1,6>& H) const;
    

    Eigen::Quaterniond smallAngleQuaternion( const Eigen::Vector3d& delta_theta) const;
};