
#include <sensor_msgs/msg/imu.hpp>
// #include <pcl_conversions/pcl_conversions.h> 

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "core/types/pcl_types.hpp"
#include "core/types/lidar_frame.hpp"

class Preprocess 
{
public: 
    Preprocess() = default;
    // CloudTPtr lidarConvert(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg);
    LidarConvertResult lidarConvert(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg);

    sensor_msgs::msg::Imu::SharedPtr imuConvert(const sensor_msgs::msg::Imu::SharedPtr& msg);

    void setPointFilterNum(int n);
    void setBlindDistance(double blind);

private: 
    bool isValidLivoxPoint(const livox_ros_driver2::msg::CustomPoint& src) const;
    bool isEveryNthPoint(uint32_t valid_num) const;
    PointT makePoint(const livox_ros_driver2::msg::CustomPoint& src) const;
    bool isTooClose(const PointT& pt) const;
    bool isDuplicatePoint(const PointT& pt, const PointT& prev_pt, bool has_prev) const;
    void finalizeCloud(const CloudTPtr& cloud) const;

private:
    int point_filter_num_ = 3;
    double blind_distance_ = 0.5;
    int n_scans_ = 6; //


};