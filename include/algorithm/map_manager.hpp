#pragma once
#include <vector>
#include <common_lib.h>
#include <third_party/ikd-Tree/ikd_Tree.h>

// IKD-Tree 기반 맵 관리자 — KD_TREE<PointType> 를 감싸는 클래스\
/*ikd-Tree의 wrapper */
//map nearest search 엔진

class MapManager
{
public:
    MapManager() = default;

    void setDownsampleParam(double size);
    bool isBuilt();
    int  validNum();
    int  treeSize();

    void build(PointVector& points);
    void nearestSearch(PointType pt, int k, PointVector& result, std::vector<float>& dist);
    int  addPoints(PointVector& points, bool rebuild);
    int  deleteBoxes(std::vector<BoxPointType>& boxes);
    //local map 밖의 map 삭제
    void acquireRemovedPoints(PointVector& out);

    // lasermapFovSegment 등 복잡한 연산을 위한 직접 접근
    KD_TREE<PointType> tree;
};
