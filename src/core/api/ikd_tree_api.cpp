#include "core/api/ikd_tree_api.hpp"

IkdTreeApi::IkdTreeApi()
{
    map_cloud_ =  std::make_shared<CloudT>();
    kdtree_ =std::make_shared<pcl::KdTreeFLANN<PointT>>();

    is_initialized_= false;
    has_params_ = false;
}

void IkdTreeApi::setParams(const SlamParams& params)
{
    params_ =params;
    has_params_ = true;

}

bool IkdTreeApi::hasFrame() const 
{
    return is_initialized_;
}

bool IkdTreeApi::isInitialized() const
{
    return is_initialized_;
}
/*
    map cloud를 kd-tree로 검색 가능한 구조로 등록하는 함수

    현재 하나의 scan point 를 map point 전체 5만개의 점과 전부 거리 계산하면 리소스 너무 크니까...
    kd-tree에서 이 point들을 공간적으로 정리해둔다. 

    x/y/z 공간 기준으로 점들을 트리 구조로 분할 -> 가까운 점을 빠르게 찾을 수 있게 준비

    비유하자면, 책 5만권이 그냥 바닥에 쌓여 있는 것 vs 분야별/위치별/번호별 정리해서 정리해놓은 것


 */
void IkdTreeApi::build(const CloudTPtr& cloud_world)
{
    if(!cloud_world || cloud_world->empty())
    {
        std::cout << "[IkdTreeApi::build[WARN] empty cloud"
                    << std::endl;

        is_initialized_ = false;
        return;
    }
    map_cloud_ = std::make_shared<CloudT>(*cloud_world);

    map_cloud_->width = static_cast<std::uint32_t>(map_cloud_->points.size());
    map_cloud_->height = 1;
    map_cloud_->is_dense = false;

    kdtree_->setInputCloud(map_cloud_); //kd-tree에 등록
    is_initialized_= true;

    std::cout << "[IkdTreeApi::build] "
            << " map_size= " << map_cloud_->size()
            << std::endl;
}

void IkdTreeApi::addPoints(const CloudTPtr& cloud_world) 
{
    if(!cloud_world || cloud_world->empty())
    {
        std::cout << "[IkdTreeApi::addPoints][WARN] empty cloud"
                    << std::endl;

        return ;
    }

    if(!is_initialized_) 
    {
        build(cloud_world);
        return;
    }

    map_cloud_->points.insert(
        map_cloud_->points.end(),
        cloud_world->points.begin(),
        cloud_world->points.end()
    );
    // 기존 map_cloud_에 새로운 cloud_world의 point들을 전부 추가한다.

    map_cloud_->width = static_cast<std::uint32_t>(map_cloud_->points.size());
    map_cloud_->height = 1;
    map_cloud_->is_dense = false;


    /*
        지금 단계에서는 nearest search 흠름 검증만 하고,  setInputCloud()로 kd-tree르 다시만들 예정
     */

    kdtree_->setInputCloud(map_cloud_);
}

/*
    가까운 map point 검색 함수
    현재 scan point 하나(point_world)를  입력으로 받아서, 이미 build된 map_cloud_안에서 가장 가까운 map point k개를 찾아주는 함수
    k: 가까운 map point를 몇개 찾을지? 보통 5개,
    nearest_points : 검색된 가까운 map point들이 저장될 vector

    squared_distances : query point와 nearest point 사이의 거리 제곱들이 저장될 vector
*/
bool IkdTreeApi::nearestSearch(
    const PointT& point_world,
    int k,
    std::vector<PointT>& nearest_points,
    std::vector<float>& squared_distances) const 
{
    nearest_points.clear();
    squared_distances.clear();

    if(!is_initialized_ || !map_cloud_ || map_cloud_->empty())
    {
        return false;
    }


    if(k <=0)       return false;


    std::vector<int> indices;
    std::vector<float> distances;

    const int found = kdtree_->nearestKSearch(
        point_world,
        k,
        indices,
        distances
    );

    if(found <=0)       return false;

    nearest_points.reserve(indices.size());
    squared_distances.reserve(distances.size());
    //거리 저장,나중에 residual 계산에 쓰일 예정

    for(std::size_t i=0; i< indices.size(); ++i) 
    {
        const int idx = indices[i];

        if(idx < 0 || idx >= static_cast<int>(map_cloud_->size()))
        {
            continue;
        }
        nearest_points.push_back(map_cloud_->points[idx]);
        squared_distances.push_back(distances[i]);
    }

    return nearest_points.size() >= static_cast<std::size_t>(k);
}


std::size_t IkdTreeApi::size() const 
{
    if(!map_cloud_)     return 0;

    return map_cloud_->size();
}
