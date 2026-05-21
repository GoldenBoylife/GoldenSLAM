#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "core/types/pcl_types.hpp"
#include "third_party/ikd_tree/ikd_tree.h"

class IkdTreeMap
{
    using IkdPointVector = KD_TREE<PointT>::PointVector;

    
public:
    IkdTreeMap();
    void clear();
    bool empty() const;
    size_t size() const;
    void insertCloud(const CloudTConstPtr& cloud_world);

    bool nearestSearch(const PointT& query_point, int k, std::vector<PointT>& nearest_points, std::vector<float>& squared_distances) ;

    CloudTPtr getDisplayMap() const;

private:
    void appendToDisplayMap(const CloudTConstPtr& cloud_world);
    void trimDisplayMapIfNeeded();
    

private:
    KD_TREE<PointT> ikd_tree_; //map 검색용 자료구조
    CloudTPtr display_map_; //보여주기 위해 누적용 map
    bool tree_initialized_ = false;
    static constexpr size_t kMaxDisplayMapPoints  = 3000000;

    
};