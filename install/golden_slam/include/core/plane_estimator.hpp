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
// + : query_point가 지금은 world 좌표계 point처럼 쓰이고 있지만, iEKF jacobian만들려면 보통 body 좌표계의 point도 필요함.
//      따라서 point_body, point_world, normal, residual함께 저장해서 iEKF update 입력 형태로 정리한다.
//      각 scan point 주변의 가까운 map point들로 local plane을 하나씩 만듬


struct ResidualCandidate
{

    Eigen::Vector3d point_body = Eigen::Vector3d::Zero();
    //scan end/body 좌표계 기준 point
    //나중에 Jacobian 계산에 사용

    Eigen::Vector3d point_world = Eigen::Vector3d::Zero();
    //world 좌표계로 변환된 point
    //현재 residual 계산에 사용


    // PointT query_point; // 디버그용
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    //map local plane 정보

    double offset = 0.0;
    //map plane에 대한 d값
    double residual = 0.0;
    //iEKF measurement에 사용할 값

    double abs_residual = 0.0;
    double plane_eig_min = 0.0;

    bool valid = false;
};

class PlaneEstimator
{
public:
    bool estimate( const std::vector<PointT>& points, EstimatedPlane& plane) const;
    
    bool buildResidualCandidate(
        const PointT& query_point_body,
        const PointT& query_point_world,
        const std::vector<PointT>& nearest_points, 
        const std::vector<float>& squared_distances,
        ResidualCandidate& candidate) const;
};