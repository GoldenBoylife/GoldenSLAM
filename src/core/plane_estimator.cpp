#include "core/plane_estimator.hpp"
static constexpr double MAX_ABS_RESIDUAL = 0.2;  // 0.3 이상 거리 되면 제외
static constexpr double MAX_PLANE_EIG_MIN = 0.01; // 0.05이상이면 평면성이 약해서 제외

/*
estimate 
    nearest point 5개 입력
    -> 중심점 centroid 계산
    -> 점들이 어느 방향으로 퍼져 있는지 covariance 계산
    -> eigen decomposition으로 normal 찾기
    -> normal과 centroid로 plane equation 계산
    -> EstimatedPlane에 결과 저장

*/

bool PlaneEstimator::estimate(const std::vector<PointT>& points, EstimatedPlane& plane) const
{
    plane = EstimatedPlane{};

    if(points.size() <3)    return false;
    //최소 점 3개 있어야 평면 구할 수 있다. 

    /*  centroid 계산
            nearest point들의 중심점 구하기. 
            이후 각 point가 이 중심점에서 얼마나 퍼져 나가는지 계산
    */
   Eigen::Vector3d centroid  = Eigen::Vector3d::Zero();

   for(const auto& p : points) 
   {
        centroid   += Eigen::Vector3d(p.x, p.y, p.z);        
   }
    centroid /= static_cast<double>(points.size());


    /*
        2. covariance 계산
            point들이 centroid 기준으로 어느 방향으로 많이 퍼져 있는지 계산한다.
    */
   Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();

   for(const auto& p : points) 
   {
        const Eigen::Vector3d q = Eigen::Vector3d(p.x, p.y, p.z) - centroid;
        
        covariance += q * q.transpose();
        // covariance : 그 방향으로 퍼진 정도
        // 평면에 가까울 수록 작음,
   }

   covariance /= static_cast<double>(points.size());


   /*
        3. eigen decomposition
        평면 위의 점들은 평면 방향으로는 많이 퍼져 있고, 
        평면 normal 방향으로는 적게 퍼져 있다. 

        수학적으로는 점3개로 normal을 구해낼수 있다.  
        하지만...
        실제는 노이즈로 인해서 완벽히 구해낼 수 없다. 
        따라서 
            p1,p2,p3로 만든 평면,
            p1,p2,p4로 만든 평면, 
            p1,p2,p5로 만든 평면
        조금씩 다를 수 있다. 그래서 5개 전체를 보고 가장 그럴듯한 평면을 찾는다.    
        평균적익 평면을 찾는 것이다. 
        그래서  
   */
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);

    if(solver.info() != Eigen::Success)     return false;

    const Eigen::Vector3d normal = solver.eigenvectors().col(0).normalized();
    // eigenvector  : 퍼진  방향
    // col(0) : 가장 작은 값, 즉 평면에 가장 가까운값에 

    /*
        4. plane equation 계산 : 평면식
            plane equation : 
                normal.dot(x) + offset =0
                다르게 표현하면 ax +by +cz +d = 0

            cetroid 는 평면 위의 대표점(p0)으로 볼 수 있으므로, 
                normal.dot(centroid) + offset =0
                다르게 표현하면 a*p0.x + b*p0.y + c*p0.z +d  =0;

            따라서, 


                offset = -normal.dot(centroid)
                다르게 표현하면 d = -(a*p0.x + b*p0.y + c*p0.z)

     */

     const double offset = -normal.dot(centroid);

     plane.normal = normal;
     plane.centroid = centroid;
     plane.offset = offset;
     plane.smallest_eigenvalue = solver.eigenvalues()(0);
     plane.valid = true;

     return true;
}

bool PlaneEstimator::buildResidualCandidate(
    const PointT& query_point_body,
    const PointT& query_point_world,
    const std::vector<PointT>& nearest_points, 
    const std::vector<float>& squared_distances,
    ResidualCandidate& candidate) const
{

    static constexpr double MAX_NEAREST_DIST = 0.8; // meter
    static constexpr double MAX_NEAREST_SQ_DIST =
        MAX_NEAREST_DIST * MAX_NEAREST_DIST;

    if (nearest_points.size() < 5)
    {
        return false;
    }

    if (squared_distances.size() >= 5 &&
        squared_distances[4] > MAX_NEAREST_SQ_DIST)
    {
        return false;
    }

    candidate = ResidualCandidate{};
    //후보 초기화

    EstimatedPlane plane;

    const bool plane_ok = estimate(nearest_points, plane);


    if(!plane_ok || !plane.valid)   return false;
    //추정 실패: plane 추정 실패

    const Eigen::Vector3d pw(query_point_world.x, query_point_world.y, query_point_world.z);
    const Eigen::Vector3d pb(query_point_body.x, query_point_body.y, query_point_body.z);

    const double residual = plane.normal.dot(pw) + plane.offset;
    //거리 차이 
    const double abs_residual = std::abs(residual);


    //추정 실패: eig_min 너무 큼, abs_residual 너무 큼
    if(plane.smallest_eigenvalue > MAX_PLANE_EIG_MIN)       return false;
    if(abs_residual > MAX_ABS_RESIDUAL)     return false;

    candidate.point_body = pb;
    candidate.point_world = pw;

    // candidate.query_point = query_point;
    candidate.normal = plane.normal;
    candidate.offset = plane.offset;
    candidate.residual = residual;
    candidate.abs_residual = abs_residual;
    candidate.plane_eig_min = plane.smallest_eigenvalue;
    candidate.valid = true;

    return true;


    
}
