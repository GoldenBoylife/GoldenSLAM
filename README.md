 Development Blog

👉 [goldenboy.co.kr](https://goldenboy.co.kr/)

# GoldenSLAM



GoldenSLAM은 LiDAR + IMU 기반 SLAM 시스템을 직접 구현하며 확장해가는 프로젝트입니다.

현재는 FAST-LIO2의 LiDAR-Inertial Odometry 구조를 참고하여  
frontend pipeline을 직접 구현하고 있습니다.

하지만 이 프로젝트의 목표는 FAST-LIO2를 그대로 재현하는 것에만 있지 않습니다.

FAST-LIO2 스타일의 빠른 odometry 구조를 기반으로 시작하되,  
이후에는 keyframe 기반 map 관리, loop closing, localization mode까지 확장하여  
하나의 완성된 SLAM 시스템으로 발전시키는 것을 목표로 합니다.

---

## Project Goal

이 프로젝트의 최종 목표는 실제 로봇에 적용 가능한 SLAM 시스템을 직접 만드는 것입니다.

특히 다음 흐름까지 이어지는 전체 구조를 목표로 합니다.

- LiDAR-Inertial Odometry
- Keyframe-based Mapping
- Loop Closing
- Global Map Optimization
- Map-based Localization
- Navigation 연동

장기적으로는 가정용 반려 로봇에 들어갈 수 있는 위치추정 및 지도작성 시스템을 만드는 것이 목표입니다.

---

## Development Direction

현재 개발 방향은 아래와 같습니다.

```text
Sensor Input
  ↓
LiDAR / IMU Synchronization
  ↓
IMU Propagation
  ↓
LiDAR Undistortion
  ↓
LiDAR-Inertial Odometry
  ↓
Keyframe Selection
  ↓
Local Map Management
  ↓
Loop Closing
  ↓
Global Map Optimization
  ↓
Localization Mode
```

---

## Current Focus

현재는 전체 구조 중 가장 앞단인 frontend pipeline을 만들고 있습니다.

특히 다음 항목들을 직접 구현하며 검증하고 있습니다.

- LiDAR preprocessing
- LiDAR frame generation
- IMU buffering
- syncMeasureGroup
- IMU propagation
- LiDAR undistortion
- pose prediction
- scan-to-map registration prototype

단순히 알고리즘을 붙이는 것이 아니라,  
각 단계의 입력과 출력이 정상인지 로그와 RViz를 통해 하나씩 확인하면서 개발하고 있습니다.

---

## Current Pipeline

```text
LiDAR callback
IMU callback
        ↓
RosBridge
        ↓
syncMeasureGroup
        ↓
IMU Propagation
        ↓
LiDAR Undistortion
        ↓
Pose Estimation
        ↓
Registration / EKF Update
        ↓
Map Update
```

---

## Roadmap

### 1. Frontend

- [x] LiDAR preprocessing
- [x] LiDAR frame 생성
- [x] IMU buffer 구성
- [x] syncMeasureGroup 구현
- [ ] imuPropagate 구현
- [ ] LiDAR undistortion 안정화
- [ ] EKF 기반 pose update
- [ ] local map 기반 scan matching

### 2. Keyframe System

- [ ] keyframe 생성 기준 설계
- [ ] keyframe pose 저장
- [ ] keyframe point cloud 저장
- [ ] local keyframe map 구성
- [ ] submap 관리 구조 설계

### 3. Loop Closing

- [ ] loop candidate 탐색
- [ ] scan context 또는 place recognition 구조 검토
- [ ] loop constraint 생성
- [ ] pose graph optimization 적용
- [ ] global map correction

### 4. Localization Mode

- [ ] mapping mode / localization mode 분리
- [ ] 저장된 map 기반 위치추정
- [ ] initial pose 설정
- [ ] map-based localization
- [ ] navigation stack 연동 준비

---

## Tech Stack

- ROS 2 Humble
- C++
- Eigen
- PCL
- Livox LiDAR
- IMU
- RViz2

---

## Development Log

개발 과정은 블로그에 정리하고 있습니다.

**Development Blog**  
https://goldenboy.co.kr/

블로그에는 단순 결과보다 다음 내용을 중심으로 기록합니다.

- 왜 실패했는지
- 어떤 로그를 보고 문제를 찾았는지
- 어떤 구조를 바꿨는지
- 다음 판단을 어떻게 했는지

GoldenSLAM은 결과물뿐 아니라,  
SLAM 시스템을 직접 쌓아가는 과정을 기록하는 프로젝트입니다.

---

## Notes

현재 프로젝트는 개발 중입니다.

FAST-LIO2의 구조를 참고하고 있지만,  
목표는 단순 재현이 아니라 keyframe, loop closing, localization까지 포함하는  
확장형 SLAM 시스템으로 발전시키는 것입니다.

개발 과정에서 가장 중요하게 보는 것은 다음과 같습니다.

- Sensor synchronization
- Data validation
- Frontend pipeline
- Debugging process
- System architecture
- Mapping / Localization 확장성

---

## Philosophy

> registration은 만능이 아니다.

SLAM은 단순히 registration 하나로 해결되는 문제가 아니라,  
센서 시간 동기화, IMU 적분, LiDAR deskew, map 관리 구조가 모두 맞물려야 한다고 생각합니다.

이 프로젝트에서는 각 단계를 직접 구현하고 검증하면서  
SLAM 시스템의 내부 구조를 하나씩 이해하고 쌓아가는 것을 목표로 합니다.
