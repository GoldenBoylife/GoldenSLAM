#include "core/preprocess.hpp"


CloudTPtr Preprocess::lidarConvert(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg)
{
    /*
    순서
        1. 점 유효성 검사
        2. 샘플링
        3. 포인트 복사
        4. 시간 저장
        5. blind 제거
        6. 중복점 제거
    */
   auto cloud = std::make_shared<CloudT>();
   cloud->points.reserve(msg->point_num); 
   //메모리 미리 확보 
   //push_back으로 반복하게되면, 수천, 수만번 재할당시 부담이 심해서 처음부터 재할당하면 성능 좋아짐.


   uint8_t valid_num =0;
   PointT prev_pt{};
   bool has_prev = false;

   for(uint32_t i =1; i< msg->point_num; ++i) 
   {
    const auto& src = msg->points[i];

    if(!isValidLivoxPoint(src)) 
    {
        continue;
    }
    ++valid_num;

    if(!isEveryNthPoint(valid_num)) {
        continue;
    }

    PointT pt = makePoint(src);
    if(isTooClose(pt)) {
        continue;
    }

    if(isDuplicatePoint(pt,prev_pt, has_prev)) {
        continue;
    }

    cloud->points.push_back(pt);
    has_prev = true;

   }

   finalizeCloud(cloud);
   return cloud;


}
bool Preprocess::isValidLivoxPoint(const livox_ros_driver2::msg::CustomPoint& src) const
{
    if(src.line >= static_cast<uint8_t>(n_scans_)) 
        return false;
    
    /*tag안에는 포인트의 품질에 대한 것이 있다.*/
    if(!(((src.tag & 0x30) == 0x10) || ((src.tag & 0x30) == 0x00)))
        return false;
    return true;
}

bool Preprocess::isEveryNthPoint(uint32_t valid_num) const
{
    return (valid_num % point_filter_num_) ==0;
}


PointT Preprocess::makePoint(const livox_ros_driver2::msg::CustomPoint& src) const
{
    PointT pt;
    pt.x = src.x;
    pt.y = src.y;
    pt.z = src.z;
    pt.intensity = static_cast<float>(src.reflectivity);

    pt.relative_time = static_cast<float>(src.offset_time) * 1e-9f;
    //offset_time이 nsec이기 때문에,  sec단위로 변환.
    //lidar는 10hz니까. 0.01마다 한 frame이나올것이고,  
    //시간 계산해보니 mid360으로부터 ns가 나오는게 맞았다. 0.0998이렇게 나옴


    return pt;
}

/*너무 가까우면 무시*/
//너무 가까우면 노이즈 or 의미없는 근거리 반사, 센서 몸체 찍거나,
bool Preprocess::isTooClose(const PointT& pt) const
{
    const double range_sq = 
        static_cast<double>(pt.x) * pt.x +
        static_cast<double>(pt.y) * pt.y +
        static_cast<double>(pt.z) * pt.z;
    return range_sq <= blind_distance_ *  blind_distance_;
}

bool Preprocess::isDuplicatePoint(const PointT& pt, const PointT& prev_pt, bool has_prev) const
{
    if(!has_prev)
        return false;
    
    //fabs: 절대값
    return  std::fabs(pt.x - prev_pt.x) <= 1e-7f  &&
            std::fabs(pt.y - prev_pt.y) <= 1e-7f  &&
            std::fabs(pt.z - prev_pt.z) <= 1e-7f  ;
}


void Preprocess::finalizeCloud(const CloudTPtr& cloud) const
{
    cloud->width = static_cast<std::uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = false;
}


sensor_msgs::msg::Imu::SharedPtr Preprocess::imuConvert(const sensor_msgs::msg::Imu::SharedPtr& msg)
{
    return msg;
}

void Preprocess::setPointFilterNum(int n)
{
    point_filter_num_ = n;
}

void Preprocess::setBlindDistance(double blind_distance)
{
    blind_distance_ = blind_distance;
}