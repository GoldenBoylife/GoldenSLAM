
#include <sensor_msgs/msg/imu.hpp>
// #include <pcl_conversions/pcl_conversions.h> 

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "core/types/pcl_types.hpp"


class Preprocess 
{
public: 
    Preprocess() = default;
    PointCloudXYZITPtr lidarConvert(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg);

    sensor_msgs::msg::Imu::SharedPtr imuConvert(const sensor_msgs::msg::Imu::SharedPtr& msg);

    void setPointFilterNum(int n);
    void setBlindDistance(double blind);

private: 
    bool isValidLivoxPoint(const livox_ros_driver2::msg::CustomPoint& src) const;
    bool isEveryNthPoint(uint32_t valid_num) const;
    PointType makePoint(const livox_ros_driver2::msg::CustomPoint& src) const;
    bool isTooClose(const PointType& pt) const;
    bool isDuplicatePoint(const PointType& pt, const PointType& prev_pt, bool has_prev) const;
    void finalizeCloud(const PointCloudXYZITPtr& cloud) const;

private:
    int point_filter_num_ = 3;
    double blind_distance_ = 0.5;
    int n_scans_ = 6; //


};