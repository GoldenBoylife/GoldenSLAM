#include "core/pointcloud_deskew.hpp"

CloudTPtr PointCloudDeskew::deskew( const LidarFrame& lidar_frame,const ImuPropagatedPoseHistory& imu_pose_history) const
{
    auto deskewed_cloud = std::make_shared<CloudT>();
    if(!lidar_frame.cloud || lidar_frame.cloud->empty())    return deskewed_cloud;

    if(imu_pose_history.empty())
    {
        std::cout << "[PointCloudDeskew] skip: empty imu pose_history" << std::endl;
        *deskewed_cloud = *lidar_frame.cloud;
        return deskewed_cloud;
    }

    deskewed_cloud->points.reserve(lidar_frame.cloud->points.size());
    /*
    scan end 기준 pose를 구한다
    기존에는 imu_pose_history.back()을 써도 되지만, 
    실제 lidar_frame.frame_end_time과 history.back().timestamp가 완전히 같지 않을 수 있다. 
    그래서 frame_end_time 기준으로 보간한 end_pose를 사용한다.
    */
    ImuPropagatedPose end_pose;
    if(!findInterpolatedPose(lidar_frame.frame_end_time, imu_pose_history, end_pose))
    {
        std::cout << "[PointCloudDeskew] skip : cannot find end pose" << std::endl;
        *deskewed_cloud = *lidar_frame.cloud;
        return deskewed_cloud;
    }

    const Eigen::Matrix3d R_end_inv = end_pose.rotation.toRotationMatrix().transpose();

    const Eigen::Vector3d t_end = end_pose.position;

    int skipped_count = 0;

    for(const auto& pt : lidar_frame.cloud->points)
    {
        /*
        livox point의 relative_time은 frame_beg_time 기준 offset임
        따라서 point_time = frame_beg_time + relative_time
        */
       const double point_time = lidar_frame.frame_beg_time + static_cast<double>(pt.relative_time);

       ImuPropagatedPose pose_i;
       if(!findInterpolatedPose(point_time, imu_pose_history, pose_i)) 
       {
        ++skipped_count ;
        continue;
       }
       const Eigen::Vector3d p_lidar( static_cast<double>(pt.x), static_cast<double>(pt.y), static_cast<double>(pt.z));

       /*
       1차 버전 가정:
       LiDAR frame = IMU/body frame

       point가 찍힌 시점의 pose_i로 world에 올린 뒤, 
       scan end pose 기준 좌표계로 다시 내린다. 

       1. p_world = R_i * p_i + t_i    
        // 로봇 기준으로 보이는 점 p_i를 world/map 기준 좌표로 바꾼다. 
        //p_i : LiDAR좌표계 기준 point 
        //R_i : point가 찍힌 시점 i의 로봇 회전값
        //t_i : 이걸 안하면, 로봇의 world위치가 반영되지 않으니까. world상에서 로봇의 위치를 추가해준다.
        예를 들어서,  
        p_i = (5,0,0)  : point한개가  로봇 기준 x방향으로 5m 앞에 점이 있다. 
        그런데 로봇이 world기준에서 회전해 있으면 로봇 기준 x방향과 world기준 x방향이 같지 않음 따라서 R_i 곱해서 world기준으로 바꿈. 
        로봇이 world기준으로 z축 기준 90도 회전해 있다고 하면, R_i* p_i = (0,5,0) 됨
        t_i = (10, 0, 0) :  로봇의 원점이 world기준 x=10위치에 있다.
        그럼 p_world = (10,5,0) 됨 
        즉 point 한개가 찍힌 시점에는, 그 점이 world기준으로 (10,5,0)에 위치해 있었다. 
        이걸 2단계에서,다시 scan end 시점 로봇 좌표계로 가져와야함.


       2. p_end = R_end^-1 * (p_world - t_end)
        // scan_end pose 기준으로 다시 내림
        //R_end : LiDAR좌표계 기준을 world로 돌리는 회전
        
        scan end 시점에 로봇이 조금 앞으로 움직였다고 가정하면, 
        첫 point가 찍힐 때 로봇의 위치는 t_i = (10,0,0)이고, 
        scan_end 시점 로봇 위치는 t_end = (10,2,0) 이라면, world기준으로 y방향 2m 움직인 상태다. 
        p_world = world기준 (10, 5, 0)
        t_end   = world기준 (10, 2, 0)
        빼면 world기준 (0,3,0)이고 scan end 시점 로봇 원점에서 봤을때 그 점은 world 좌표 방향으로 (0,3,0) 만큼 떨어져 있다. 즉 world기준 로봇보다 y방향 3m에 이 point가 있다. 
        R_end는 body x축 -> world y축이라면
        R_end^-1은 world y축 -> LiDAR x축 으로 바꾼다. 
        world기준 (0,3,0)인데 이걸 LiDAR기준 (3,0,0)으로 바꾼다. 
        즉... 
        scan end 기준으로 보정된 point위치다. 
        처음에는 앞 5m였던 점이, 로봇이 scan 중 2m 앞으로 움직였기 때문에, scan end기준에서는 앞 3m로 보정된 것입니다. 




       각  point를 scan 끝 시점 좌표계로 다시 맞춤
       그래서 각 point를 "자기가 찍힌 시점의 로봇 pose"로 먼저 world에 올린 다음, 
       다시 "scan end 시점의 로봇 좌표계"로 가져온다. 

       더 자세히: 각 point는 자기 찍힌 시점의 LiDAR 좌표계에서 맞는 좌표다. 그런데 scan 안의 point들이 서로 다른 시점의 로봇 좌표계 기준이라서, 하나의 cloud로 묶으면 distorted로 보임.
                그래서, point하나하나를 움직여서 scan end시점의 LiDAR 좌표계 기준으로 통일 한다. 
            
       지금은 step2결과가 LiDAR 기준 좌표지만, 나중에는 결국 world기준으로 바꿔야 한다. 
       하지만 deskew의 목적은  "한 scan 내부의 시간 왜곡을 제거하는 작업"이기에 world변환은 나중에 하도록 분리시켜놓는다. 
       */
        const Eigen::Vector3d p_world = pose_i.rotation * p_lidar + pose_i.position;
        //p_world : world기준 점의 위치 
        const Eigen::Vector3d p_deskewed = R_end_inv * (p_world - t_end);
        //p_deskewed : Scan end 시점에서 LiDAR기준  점의 위치
        //점 하나하나를 옮기고 있다. 


        PointT q = pt;
        //여기서 intensity와 relative_time이 복사됨, 다만 아래서 pose만 바뀌는 것
        q.x = static_cast<float>(p_deskewed.x());
        q.y = static_cast<float>(p_deskewed.y());
        q.z = static_cast<float>(p_deskewed.z());

        deskewed_cloud->points.push_back(q);
        
    }
    deskewed_cloud->width = static_cast<std::uint32_t>(deskewed_cloud->points.size());
    deskewed_cloud->height  =1;
    deskewed_cloud->is_dense = false;


    // std::cout
    //     << "[PointCloudDeskew]"
    //     << " input=" << lidar_frame.cloud->points.size()
    //     << " output=" << deskewed_cloud->points.size()
    //     << " skipped=" << skipped_count
    //     << " pose_count=" << imu_pose_history.size()
    //     << std::endl;

    return deskewed_cloud;


}

/*
어떤 LiDAR point가 찍힌 시간 query_time에 로봇 pose가 어디였는지 찾아 주는 함수
LiDAR 한 프레임안에서도 point마다 찍힌 시간이조금씩 다르다. 
    point A : t = 10.001
    point B : t = 10.035
    point C : t = 10.087

그런데 IMU propagation pose history는 이런식으로 띄엄띄엄 있다. 

pose0 : t = 10.000
pose1 : t = 10.005
pose2 : t = 10.010
pose3 : t = 10.015


그럼 query_time = 10.007일때 정확히 같은 pose는 없는데,  어떤 pose일까? 이것을 만드는 함수다.

history안에 쌓여 있는 IMU propagation pose 중에서, query_time 에  해당하는 로봇 pose를 찾아서 없으면 앞뒤 pose 사이를 보간해서 out_pose에 넣는다.
*/
bool PointCloudDeskew::findInterpolatedPose(double query_time, const ImuPropagatedPoseHistory& history, ImuPropagatedPose& out_pose) const
{
    if(history.empty()) return false;

    if(history.size() == 1)
    {
        out_pose = history.front();
        out_pose.timestamp = query_time;
        return true;
    }

    /* 
        query_time이 history 범위 밖인 경우,
        현재 syncMeasure 구조에서는 LiDAR frame 시작 /끝보다 imu pose history가 살짝 짧을 수 있다. 

        1차 버전에서는 clamp 처리한다. 나중에 lidar_beg 직전 IMU, lidar_end 직후 IMU까지 포함하면 더 정확해진다.
    */

    /*clamp 처리 : 값이 허용 범위 밖으로 나가면, 가장 가까운 경계값으로 강제로 저장 */
    if(query_time <= history.front().timestamp) 
    {
        out_pose = history.front();
        out_pose.timestamp = query_time;
        return true;
    }
    if(query_time >= history.back().timestamp) 
    {
        out_pose = history.back();
        out_pose.timestamp = query_time;
        return true;
    }

    
    /*
    interpolation 보간 단계 
    query_time보다 크거나 같은 첫 pose를 찾는다. history는 timestamp오름차순이어야 함
    lower_bound로 query_time 바로 뒤 pose 찾기.
    만약 timestamp가 아래와 같다면, 
        history[0].timestamp = 10.00
        history[1].timestamp = 10.01
        history[2].timestamp = 10.02
        history[3].timestamp = 10.03
        history[4].timestamp = 10.04

        query_time이 10.025일때 lower_bound가 찾는 것은 10.03이다.
    */
    auto upper_it = std::lower_bound(
        history.begin(),
        history.end(),
        query_time,
        [](const ImuPropagatedPose& pose, double t)
        {
            return pose.timestamp < t;
        }
    );

    /*이미 앞서서 처리 했지만, timestamp가 이상하게 꼬였을 경우를 대비한 방어 코드 */
    if(upper_it == history.begin()) 
    {
        out_pose = history.front();
        out_pose.timestamp = query_time;
        return true;
    }

    if(upper_it == history.end())
    {
        out_pose = history.back();
        out_pose.timestamp = query_time;
        return true;
    }

    /*
    query_time 앞뒤 pose 잡기
        10.02           10.025           10.03
        p0       query_time              p1
        |------------|-------------------|
    
    */
    const auto& p1 = *upper_it;
    const auto& p0 = *(upper_it -1);

    const double dt = p1.timestamp - p0.timestamp;

    /*dt가 0이 나오면 안되는데 이상하면 p0사용 */
    if(dt <= 1e-9)
    {
        out_pose = p0;
        out_pose.timestamp = query_time;
        return true;
    }

    /*
    alpha : query_time이 p0와 p1 사이 몇퍼센트 지점에 있는지를 의미함 
        10.02           10.025           10.03
        p0              50%              p1
        |---------------|----------------|
                    alpha = 0.5
    
    */


    /*alpha도 clamp 처리*/
    double alpha = (query_time -p0.timestamp) /dt;

    if(alpha <0.0)
    {
        alpha = 0.0;
    } 
    else if(alpha > 1.0)
    {
        alpha = 1.0;
    }

    out_pose.timestamp = query_time;

    //position은 선형 보간
    out_pose.position = p0.position + alpha * (p1.position - p0.position);

    //velocity도 일단 선형 보간
    out_pose.velocity = p0.velocity + alpha * (p1.velocity - p0.velocity);

    //rotation은 quaternion slerp 보간
    out_pose.rotation = p0.rotation.slerp(alpha, p1.rotation).normalized();

    return true;

}
