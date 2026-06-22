# GoldenSLAM

FAST-LIO2 구조를 기반으로 LiDAR-Inertial SLAM 파이프라인을 직접 재구현하고 있는 개인 개발 프로젝트입니다.

이 프로젝트의 목표는 단순히 기존 오픈소스를 실행하는 것이 아니라, LiDAR 전처리, IMU propagation, LiDAR undistortion, map matching, residual 계산, iEKF correction, 그리고 이후 keyframe 기반 backend optimization까지 SLAM 전체 구조를 단계별로 직접 이해하고 구현하는 것입니다.

## Project Motivation

저는 이전 직장에서 실외 자율주행 로봇을 위한 LiDAR SLAM / Localization 관련 개발을 수행했습니다.
당시에는 실외 주행 환경에서 map 생성, map 정합, localization, AprilTag 기반 보정, RViz/Qt 기반 map editor 개발 등을 경험했습니다.

GoldenSLAM은 그 경험을 바탕으로, SLAM 기술을 더 최신 구조로 다시 공부하고 확장하기 위한 프로젝트입니다.

특히 FAST-LIO2와 같은 최신 LiDAR-Inertial SLAM 구조를 직접 분석하고, 제 코드로 다시 구성하면서 다음 역량을 쌓는 것을 목표로 합니다.

```text id="r6wzwd"
- LiDAR-Inertial SLAM 구조 이해
- IMU propagation / iEKF prediction 구현
- LiDAR undistortion / deskew 구현
- map matching / residual / Jacobian 구조 이해
- ikd-tree 기반 map update 구조 구현
- keyframe 기반 backend 추가
- pose graph optimization 추가
- node 기반 map 관리 구조 설계
```

장기적으로는 다음 방향으로 확장하는 것을 목표로 하고 있습니다.

```text id="is6bx9"
LiDAR SLAM
→ LiDAR-Inertial SLAM
→ Keyframe-based SLAM Backend
→ VSLAM / Visual-Inertial SLAM
→ Robot Localization
→ Spatial AI / Physical AI
```

## Current Branch

현재 공개 버전은 `dev_v4` 기준입니다.

이 코드는 GoldenSLAM의 4번째 재구현 시도입니다.

이전 1~3차 시도에서는 FAST-LIO2 구조를 따라 iEKF pose correction까지 적용하려 했으나, correction 이후 pose drift 및 trajectory 불안정 문제가 발생했습니다.

그래서 dev_v4에서는 문제를 다시 처음부터 분해했습니다.

```text id="zj9a55"
1. LiDAR / IMU synchronization
2. IMU initialization
3. IMU propagation
4. Pose history 저장
5. Raw cloud publish
6. Undistort cloud publish 구조 구성
7. IMU predicted pose 기반 debug map 적층
8. LiDAR point-wise undistortion
9. ikd-tree map matching
10. residual / Jacobian 계산
11. iEKF correction 재진입
```

현재는 iEKF correction을 성급하게 붙이기보다, FAST-LIO2의 front-end pipeline을 단계별로 검증하는 방식으로 개발하고 있습니다.

## Current Status

현재까지 구현 및 검증한 내용은 다음과 같습니다.

```text id="bwa4dx"
Completed
- ROS2 기반 GoldenSLAM node 구조 구성
- LiDAR / IMU buffer 관리
- LiDAR frame과 IMU measurement synchronization
- IMU 평균 기반 초기화
- IMU propagation
- frame별 pose history 저장
- raw cloud publish
- undistort cloud publish 구조 추가
- IMU predicted pose 기반 debug map 적층
- RViz 기반 cloud / debug map 검증

In Progress
- LiDAR point-wise undistortion
- pose history 기반 deskew 구현
- FAST-LIO2 방식의 map matching 구조 정리

Planned
- voxel downsampling
- ikd-tree map insert / nearest search
- plane residual 계산
- iEKF correction
- keyframe 추가
- backend optimization 추가
- node 기반 map 저장 및 관리
- localization / mapping 모드 분리
```

## Architecture

현재 dev_v4 구조는 다음과 같은 방향으로 정리하고 있습니다.

```text id="fdqxs8"
GoldenSlamApp
 └── RosBridge
      ├── LiDAR Subscriber
      ├── IMU Subscriber
      ├── Cloud Publisher
      ├── Odometry Publisher
      └── Debug Map Publisher

SlamCore
 ├── Preprocess
 ├── ImuProcessor
 │    ├── IMU initialization
 │    ├── IMU propagation
 │    ├── pose history
 │    └── LiDAR undistortion
 │
 ├── EsekfomApi
 │    ├── state management
 │    ├── IMU prediction
 │    └── future iEKF update
 │
 ├── IkdTreeApi
 │    ├── future map insert
 │    └── future nearest search
 │
 └── Future Backend
      ├── keyframe management
      ├── pose graph
      ├── optimization
      └── node-based map management
```

FAST-LIO2의 구조를 최대한 따르되, 원본의 복잡한 함수 포인터 기반 구조는 역할별 클래스로 분리하여 읽기 쉽게 재구성하는 방향을 목표로 합니다.

```text id="bsfv2p"
FAST-LIO2 original          GoldenSLAM dev_v4 direction

get_f()                     ImuMotionModel / EsekfomApi predict
df_dx()                     State Jacobian module
df_dw()                     Noise Jacobian module
h_share_model()             LidarMeasurementModel
ikd-tree search             IkdTreeApi
plane residual              PlaneEstimator / ResidualBuilder
iEKF update                 EsekfomApi::updateLidar()
```

## Future Roadmap

GoldenSLAM의 최종 목표는 단순한 frame-to-map LiDAR-Inertial odometry를 넘어서, keyframe과 backend를 가진 SLAM 구조로 확장하는 것입니다.

향후 목표는 다음과 같습니다.

```text id="ch5wu6"
1. Frontend 안정화
   - LiDAR undistortion
   - ikd-tree 기반 map matching
   - residual / Jacobian 계산
   - iEKF correction 안정화

2. Keyframe 기반 구조 추가
   - 일정 거리 / 회전 변화 기준 keyframe 생성
   - keyframe pose 저장
   - keyframe cloud 저장
   - local map 구성

3. Backend optimization 추가
   - pose graph 구성
   - keyframe 간 constraint 생성
   - loop closure 또는 map correction 확장 가능 구조 설계
   - g2o / GTSAM 기반 optimization 적용

4. Node 기반 map 관리
   - map을 하나의 거대한 point cloud로만 관리하지 않고,
     keyframe 또는 node 단위로 저장
   - node별 pose, cloud, timestamp, connection 정보 관리
   - map update / map merge / localization에 활용 가능한 구조 설계

5. Localization 모드 확장
   - mapping 결과를 저장
   - 저장된 map에서 localization 수행
   - map node 기반 partial update 구조 검토
```

즉, GoldenSLAM은 다음 구조를 목표로 합니다.

```text id="evlr4s"
LiDAR-Inertial Frontend
        ↓
Keyframe Selection
        ↓
Local Map / ikd-tree Update
        ↓
Pose Graph Backend
        ↓
Optimization
        ↓
Node-based Map Management
        ↓
Localization / Map Update
```

## Development Log

개발 과정은 개인 개발 블로그에 정리하고 있습니다.

* Blog: https://goldenboy.co.kr

개발 로그에서는 단순 결과물보다, SLAM 시스템을 이해하고 재구성하는 과정을 중요하게 기록하고 있습니다.

주요 기록 주제는 다음과 같습니다.

```text id="idwm20"
- FAST-LIO2 구조 분석
- LiDAR / IMU synchronization
- IMU propagation
- pose history
- LiDAR undistortion
- iEKF prediction / correction
- map matching
- ikd-tree 구조
- keyframe / backend 확장 계획
- SLAM debugging 과정
```

## What I Want to Show

이 프로젝트를 통해 보여주고 싶은 것은 단순히 SLAM 코드를 실행할 수 있다는 것이 아닙니다.

제가 보여주고 싶은 것은 다음입니다.

```text id="fq1m3m"
- SLAM 구조를 직접 분석하고 재구성하는 능력
- C++ / ROS2 기반 로봇 시스템 개발 능력
- LiDAR / IMU sensor fusion pipeline 이해
- FAST-LIO2와 같은 최신 SLAM 구조를 학습하고 적용하는 과정
- 실패한 구조를 분석하고 다시 설계하는 태도
- frontend에서 backend까지 확장 가능한 SLAM 시스템 설계 방향
- 장기적으로 SLAM / VSLAM / Spatial AI로 성장하려는 개발 방향성
```

GoldenSLAM은 아직 완성된 production-ready SLAM system은 아닙니다.
하지만 SLAM 개발자로서 최신 구조를 다시 공부하고, 직접 구현하고, 실패한 부분을 분석하며, 더 나은 구조로 확장해 나가는 과정 자체를 보여주기 위한 프로젝트입니다.
