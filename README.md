history는 개인블로그에서 확인 : goldenboy.co.kr 

# GoldenSLAM

LiDAR-IMU 기반 SLAM을 직접 다시 구현해보는 개인 프로젝트입니다.  

FAST-LIO2의 흐름을 참고하되, 구조를 그대로 복붙하는 것이 아니라 **직접 이해하고 내 방식으로 다시 설계하는 것**을 목표로 하고 있습니다.

현재는 frontend 초기 단계로,

- sensor input
- LiDAR preprocess
- IMU / LiDAR sync
- MeasureGroup 생성
- IMU initialization
- IMU prediction
- rotation-only LiDAR deskew
- RViz debug visualization

까지 진행했습니다.

---

## Project Goal

이 프로젝트의 목적은 단순히 SLAM을 “돌려보는 것”이 아니라,  
**LiDAR-IMU SLAM의 frontend와 backend를 직접 다시 구현하면서 구조와 동작 원리를 깊게 이해하는 것**입니다.

장기적으로는 아래 단계까지 가는 것을 목표로 합니다.

- frontend 직접 구현
- scan-to-map pose estimation
- local map 관리
- keyframe 저장
- graph-based backend
- loop closing
- localization mode
- 실제 로봇(GoldenBot)에 적용

---

## Current Progress

### Done
- [x] ROS2 기반 프로젝트 구조 구성
- [x] Livox MID-360 입력 수신
- [x] LiDAR preprocess
- [x] IMU / LiDAR 버퍼 적재
- [x] `MeasureGroup` 생성
- [x] `processMeasure()` 흐름 정리
- [x] IMU initialization
- [x] IMU prediction
- [x] deskew pose 기록
- [x] rotation-only LiDAR undistortion
- [x] RViz에서 preprocess / undistorted cloud 비교

### In Progress
- [ ] basic pose estimation
- [ ] point cloud accumulation
- [ ] local map visualization

### Planned
- [ ] scan-to-map registration
- [ ] keyframe management
- [ ] graph-based backend
- [ ] loop closure
- [ ] map save / localization mode
- [ ] GoldenBot 실기 적용

---

## Frontend Flow

현재 frontend 초기 흐름은 아래와 같습니다.

```text
LiDAR / IMU input
    ↓
Preprocess
    ↓
MeasureGroup sync
    ↓
processMeasure()
    ├─ input validation
    ├─ IMU initialization
    └─ runFrontend()
         ├─ predictByImu()
         └─ undistortLidar()
````

---

## What is implemented now?

### 1. MeasureGroup

LiDAR 한 프레임과 그 프레임 구간에 해당하는 IMU 묶음을 하나의 처리 단위로 만듭니다.

### 2. IMU Initialization

시스템 시작 직후 바로 propagation에 들어가지 않고,
초기 몇 프레임 동안 평균 가속도와 평균 자이로를 누적하여 IMU 초기 상태를 잡습니다.

### 3. IMU Prediction

현재 LiDAR 프레임 구간 동안 IMU를 시간 순서대로 적분합니다.
이 과정에서 단순히 최종 pose만 얻는 것이 아니라,
**frame 내부 각 시점의 pose를 기록**하여 이후 deskew에 사용합니다.

### 4. LiDAR Undistortion

LiDAR 한 프레임 안의 각 point는 서로 다른 시각에 찍히기 때문에,
각 point의 `relative_time`에 대응하는 pose를 찾아
**frame end 기준으로 점을 재배치**합니다.

현재는 translation까지 모두 넣은 full deskew가 아니라,
**rotation-only deskew**를 먼저 검증한 상태입니다.

---

## Why rotation-only deskew first?

처음에는 rotation + translation을 모두 적용한 full deskew를 시도했습니다.
하지만 현재 단계의 `pred_pos_`는 아직

* bias 보정 없음
* gravity 정교화 전
* scan-to-map 보정 없음

상태에서 얻은 값이라, translation까지 포함하면 point cloud가 오히려 불안정하게 흔들렸습니다.

그래서 현재는 우선

* rotation deskew 먼저 검증
* translation deskew는 다음 단계 이후 재도입

이라는 방향으로 진행 중입니다.

---

## RViz Debug Result

현재는 아래 두 cloud를 같은 frame pair 기준으로 RViz에서 비교하고 있습니다.

* `/debug/preprocess_lidar`
* `/debug/undistorted_lidar`

현재 결과는 두 cloud가 거의 비슷한 형상을 유지하면서 안정적으로 겹치는 상태입니다.
이것은 지금 구간에서 왜곡이 아주 크지 않았을 수도 있고,
현재 rotation-only deskew가 과하게 깨지지 않고 정상 방향으로 동작하고 있다는 뜻으로 해석하고 있습니다.

---

## Tech Stack

* **Language**: C++17
* **Middleware**: ROS2 Humble
* **Point Cloud**: PCL
* **Math**: Eigen
* **Sensor**: Livox MID-360
* **Visualization**: RViz2

---

## Project Structure

```text
GoldenSLAM/
├─ include/
│  ├─ app/
│  ├─ core/
│  │  ├─ types/
│  │  ├─ preprocess.hpp
│  │  └─ slam_core.hpp
│  └─ platform/
│     └─ ros_bridge.hpp
│
├─ src/
│  ├─ app/
│  ├─ core/
│  │  ├─ preprocess.cpp
│  │  └─ slam_core.cpp
│  └─ platform/
│     └─ ros_bridge.cpp
│
├─ launch/
├─ config/
├─ rviz/
└─ CMakeLists.txt
```

---

## How to Run

### 1. Build

```bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch

```bash
ros2 launch golden_slam golden_slam.launch.py
```

### 3. RViz

RViz config 파일은 `rviz/` 폴더 안의 설정 파일을 사용합니다.

---

## Roadmap

### Step 1. Frontend refinement

* IMU prediction 안정화
* deskew 개선
* pose estimation 추가
* local map update

### Step 2. Backend

* keyframe 저장
* graph node / edge 구성
* loop detection
* graph optimization

### Step 3. Localization / deployment

* map save
* localization mode
* GoldenBot 적용

---

## Why this project?

회사에서 SLAM 관련 작업을 하면서 느낀 점은,
**기존 오픈소스를 “쓰는 것”과 처음부터 다시 “만드는 것”은 완전히 다르다**는 것이었습니다.

그래서 이 프로젝트는 단순히 FAST-LIO2를 실행하는 것이 아니라,
그 구조와 개념을 바탕으로 **내 구조로 다시 구현해보는 과정**에 더 가깝습니다.

---

## Status

현재 프로젝트는 아직 개발 중이며,
frontend를 하나씩 직접 구현하면서 점진적으로 확장하고 있습니다.

즉 지금은 “완성된 SLAM 시스템”이라기보다,
**SLAM을 직접 만들면서 구조를 검증하고 쌓아가는 개발 기록형 프로젝트**입니다.

