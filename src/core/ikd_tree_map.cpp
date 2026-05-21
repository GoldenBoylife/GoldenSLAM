#include "core/ikd_tree_map.hpp"


//alias 필요: PointT는 Eigen 정렬이 필요한 타입이어서, 

IkdTreeMap::IkdTreeMap() 
{
    /*내부에 map cloud 저장*/
    display_map_ = std::make_shared<CloudT>();
    
    display_map_->width =0;
    display_map_->height = 1;
    display_map_->is_dense = false;
}

void IkdTreeMap::clear() 
{
    if(!display_map_) 
        display_map_ =std::make_shared<CloudT>();
    

    display_map_->clear();
    display_map_->width = 0;
    display_map_->height = 1;
    display_map_->is_dense =false;

    /*
    TODO: 실제 ikd-tree claer API 확인 후 교체,
    지금은 tree_initailized만 falase로 돌려서, 
    다음 insertCloud에서 Build()를 다시 호출
    */
    tree_initialized_ =false;

}

bool IkdTreeMap::empty() const 
{
    return !tree_initialized_ || size() == 0;
}

size_t IkdTreeMap::size() const 
{
    if(!display_map_) return 0;
    return display_map_->points.size();
}

/*
    world좌표계 cloud를 여기에 insert한다. 
*/
void IkdTreeMap::insertCloud(const CloudTConstPtr& cloud_world) 
{
    if(!cloud_world || cloud_world->empty()) return;

    IkdPointVector points_to_add;
    points_to_add.reserve(cloud_world->points.size());

    for(const auto& point : cloud_world->points)
    {
        points_to_add.push_back(point);
    }

    if(points_to_add.empty())   return;

    if(!tree_initialized_) 
    {
        ikd_tree_.Build(points_to_add);
        tree_initialized_ = true;

    }
    else 
    {
        constexpr bool downsample_on = false;
        ikd_tree_.Add_Points(points_to_add, downsample_on);
    }
    appendToDisplayMap(cloud_world);
}
/*query_point 주변에서 ikd-tree map 안에 있는 가까운 point k개를 찾는다.*/
bool IkdTreeMap::nearestSearch(const PointT& query_point, int k, std::vector<PointT>& nearest_points, std::vector<float>& squared_distances) 
{
    nearest_points.clear();
    squared_distances.clear();
    //기존 map 을 지우는 게 아니라, 검색 결과를 담을 출력 벡터를 비움.

    if(!tree_initialized_ || k <=0) return false;
    //tree 준비 안되었거나 k가 이상하면 실패 ,왜냐면 검색할 대상이 없으니

    IkdPointVector search_result;
    std::vector<float> search_distances;

    /*
        ikd-tree의 Nearest Search는 squared distance를 반환한다.
        즉 sqrt를 씌우기 전 거리값이다.
    */
   ikd_tree_.Nearest_Search(query_point,k,search_result, search_distances);
   //ikd-tree 안에 들어 있는 map point들 중에서 query_point와 가까운 point k개 찾기


   if(search_result.empty())    return false;

   nearest_points.assign(search_result.begin(), search_result.end());
   squared_distances = search_distances;

   return true; 
}

CloudTPtr IkdTreeMap::getDisplayMap() const 
{
    if(!display_map_) 
    {
        return std::make_shared<CloudT>();
    }

    /*
        외부에서 display_map_ 원본을 직접 건드리지 않도록 복사본 반환,
        RosBridge publish 중 내부 map이 바뀌는 문제를 줄일 수있다. 
    */
    return std::make_shared<CloudT>(*display_map_);

}

void IkdTreeMap::appendToDisplayMap(const CloudTConstPtr& cloud_world)
{
    if(!cloud_world || cloud_world->empty()) return;
    if(!display_map_)   
        display_map_ = std::make_shared<CloudT>();

    *display_map_ += *cloud_world;

    trimDisplayMapIfNeeded();

    display_map_->width = static_cast<std::uint32_t>(display_map_->points.size());

    display_map_->height = 1;
    display_map_->is_dense = false;

}



void IkdTreeMap::trimDisplayMapIfNeeded() 
{
    if(!display_map_) return;

    if(display_map_->points.size() <= kMaxDisplayMapPoints) return;

    const size_t remove_count = display_map_->points.size() - kMaxDisplayMapPoints;

    display_map_->points.erase( display_map_->points.begin(),  display_map_->points.begin() + static_cast<std::ptrdiff_t>(remove_count) );


}