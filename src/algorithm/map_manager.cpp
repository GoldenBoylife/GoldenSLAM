#include "algorithm/map_manager.hpp"

void MapManager::setDownsampleParam(double size)
{
    tree.set_downsample_param(size);
}

bool MapManager::isBuilt()
{
    return tree.Root_Node != nullptr;
}

int MapManager::validNum()
{
    return tree.validnum();
}

int MapManager::treeSize()
{
    return tree.size();
}

void MapManager::build(PointVector& points)
{
    tree.Build(points);
}

void MapManager::nearestSearch(PointType pt, int k, PointVector& result, std::vector<float>& dist)
{
    tree.Nearest_Search(pt, k, result, dist);
}

int MapManager::addPoints(PointVector& points, bool rebuild)
{
    return tree.Add_Points(points, rebuild);
}

int MapManager::deleteBoxes(std::vector<BoxPointType>& boxes)
{
    return tree.Delete_Point_Boxes(boxes);
}

void MapManager::acquireRemovedPoints(PointVector& out)
{
    tree.acquire_removed_points(out);
}
