#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

/*PCL이 요구하는 매크로가 붙음*/
//PCL_ADD_POINT4D : x,y,z넣는게 아니라, 정렬 최적화 위해서 씀
//EIGEN_ALIGN16 : CPU 선호하는 메모리 정렬, 속도,안정성
//EIGEN_MAKE_ALIGNED_OPERATOR_NEW: new로 선언해도 정렬 안깨지도록 
struct EIGEN_ALIGN16 PointT 
{
    PCL_ADD_POINT4D; //quad-word XYZ
    float intensity;
    float relative_time; //point relative time
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};




using CloudT = pcl::PointCloud<PointT>; //포인트 여러개 모인것
using CloudTPtr = std::shared_ptr<CloudT>;
using CloudTConstPtr = std::shared_ptr<const CloudT>;
//std::shared_ptr<const PointCloudXYZIT> : 읽기 전용 객체 포인터



/*pcl에게 내가 만든 커스텀 struc에 이런 필드가 있다고 알려주는 것*/
//POINT_CLOUD_REGISTER_POINT_STRUCT : 등록 매크로
//PointXYZIT 이건 내가 만든 타입이고, 아래와 같은 필드가 있어. 
//타입은 float이고, struct 맴버 이름은 relative_time이고, pcl에 등록되는 필드 이름도 relative_time
POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, relative_time, relative_time)
)