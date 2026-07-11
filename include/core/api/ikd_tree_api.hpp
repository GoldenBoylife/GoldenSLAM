#pragma once

#include <deque>
#include <cstddef>
#include <vector>
#include <iostream>



#include <pcl/kdtree/kdtree_flann.h>

#include "core/types/slam_types.hpp"
// #include "core/types/lidar_frame.hpp"


/*
    world frame 기준 map cloud를 저장하고, 그 map cloud에서 현재 scan point 주변으니 가까운 point들을 빠르게 찾기 위한 클래스

*/
class IkdTreeApi
{
public:
    IkdTreeApi();
    ~IkdTreeApi() = default;


    void setParams(const SlamParams& params);
    // void pushLidarFrame(const LidarFrame& frame);
    bool hasFrame() const;
    // LidarFrame popFrame();
    bool isInitialized() const;

    void build(const CloudTPtr& cloud_world);
    //map cloud를 kd-tree 검색 가느한 구조로 등록하는 함수


    void addPoints(const CloudTPtr& clou_world);
    bool nearestSearch(
        const PointT& point_world,
        int k,
        std::vector<PointT>& nearest_points,
        std::vector<float>& squared_distances
    ) const;

    std::size_t size() const;

public://params

private:



private: //params

    bool has_params_ =false;
    // std::deque<LidarFrame> lidar_frames_;

    SlamParams params_;

    bool is_initialized_ = false;

    CloudTPtr map_cloud_;
    pcl::KdTreeFLANN<PointT>::Ptr kdtree_;

};