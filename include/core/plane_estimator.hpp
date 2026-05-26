#pragma once

#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "core/types/pcl_types.hpp"

struct EstimatedPlane
{
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero(); 
    //5개 점들의 중심점, 평균위치

    /*
    plane equation :
        normal.dot(x) + offset = 0
    */
   double offset = 0.0;

   double smallest_eigenvalue = 0.0;
   //평면 추정 품질 확인용

   bool valid = false;
};



/*scan point 하나에 대하여 만들어진 residual 후보 1개 */
struct ResidualCandidate
{
    PointT query_point;
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();

    double offset = 0.0;
    double residual = 0.0;
    double abs_residual = 0.0;
    double plane_eig_min = 0.0;

    bool valid = false;
};
static constexpr double MAX_ABS_RESIDUAL = 1.0;  // 1.0 이상 거리 되면 제외
static constexpr double MAX_PLANE_EIG_MIN = 0.05; // 0.05이상이면 평면성이 약해서 제외

class PlaneEstimator
{
public:
    bool estimate( const std::vector<PointT>& points, EstimatedPlane& plane) const;
    
    bool buildResidualCandidate(const PointT& query_point, const std::vector<PointT>& nearest_points, ResidualCandidate& candidate) const;
};