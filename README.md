
# GoldenSLAM V1 구조 설계
## 목표
- core는 ROS/QT 비의존적이게
- platform/ros는 ROS2 입출력, 타이머, 서비스 담당
- ui는 QT 기반 시각화 제어
- app은 전체 조립과 실행 담당

파일 구조 
GoldenSLAM/
├─ CMakeLists.txt
├─ package.xml
├─ README.md
├─ config/
│  ├─ mid360.yaml
│  └─ app.yaml
├─ launch/
│  └─ golden_slam.launch.py
├─ include/golden_slam/
│  ├─ app/
│  │  ├─ golden_slam_app.hpp
│  │  └─ app_context.hpp
│  │
│  ├─ core/
│  │  ├─ slam_core.hpp
│  │  │
│  │  ├─ types/
│  │  │  ├─ pose.hpp
│  │  │  ├─ transform.hpp
│  │  │  ├─ state.hpp
│  │  │  ├─ lidar_frame.hpp
│  │  │  ├─ imu_packet.hpp
│  │  │  ├─ measure_group.hpp
│  │  │  ├─ keyframe.hpp
│  │  │  ├─ frontend_result.hpp
│  │  │  └─ map_snapshot.hpp
│  │  │
│  │  ├─ frontend/
│  │  │  ├─ sensor_data_buffer.hpp
│  │  │  ├─ measurement_synchronizer.hpp
│  │  │  ├─ lidar_preprocessor.hpp
│  │  │  ├─ imu_initializer.hpp
│  │  │  ├─ scan_deskewer.hpp
│  │  │  ├─ lio_estimator.hpp
│  │  │  ├─ measurement_model_builder.hpp
│  │  │  ├─ local_map.hpp
│  │  │  ├─ keyframe_selector.hpp
│  │  │  └─ frontend_pipeline.hpp
│  │  │
│  │  ├─ backend/
│  │  │  ├─ keyframe_database.hpp
│  │  │  ├─ pose_graph.hpp
│  │  │  ├─ loop_detector.hpp
│  │  │  ├─ graph_optimizer.hpp
│  │  │  ├─ global_map.hpp
│  │  │  └─ backend_pipeline.hpp
│  │  │
│  │  └─ io/
│  │     ├─ config_loader.hpp
│  │     ├─ map_saver.hpp
│  │     └─ keyframe_saver.hpp
│  │
│  ├─ platform/
│  │  └─ ros/
│  │     ├─ ros_bridge.hpp
│  │     ├─ ros_message_converter.hpp
│  │     └─ ros_publisher.hpp
│  │
│  └─ ui/
│     ├─ main_window.hpp
│     ├─ map_view_widget.hpp
│     └─ status_panel.hpp
│
├─ src/
│  ├─ app/
│  │  ├─ main.cpp
│  │  └─ golden_slam_app.cpp
│  │
│  ├─ core/
│  │  ├─ slam_core.cpp
│  │  │
│  │  ├─ frontend/
│  │  │  ├─ sensor_data_buffer.cpp
│  │  │  ├─ measurement_synchronizer.cpp
│  │  │  ├─ lidar_preprocessor.cpp
│  │  │  ├─ imu_initializer.cpp
│  │  │  ├─ scan_deskewer.cpp
│  │  │  ├─ lio_estimator.cpp
│  │  │  ├─ measurement_model_builder.cpp
│  │  │  ├─ local_map.cpp
│  │  │  ├─ keyframe_selector.cpp
│  │  │  └─ frontend_pipeline.cpp
│  │  │
│  │  ├─ backend/
│  │  │  ├─ keyframe_database.cpp
│  │  │  ├─ pose_graph.cpp
│  │  │  ├─ loop_detector.cpp
│  │  │  ├─ graph_optimizer.cpp
│  │  │  ├─ global_map.cpp
│  │  │  └─ backend_pipeline.cpp
│  │  │
│  │  └─ io/
│  │     ├─ config_loader.cpp
│  │     ├─ map_saver.cpp
│  │     └─ keyframe_saver.cpp
│  │
│  ├─ platform/
│  │  └─ ros/
│  │     ├─ ros_bridge.cpp
│  │     ├─ ros_message_converter.cpp
│  │     └─ ros_publisher.cpp
│  │
│  └─ ui/
│     ├─ main_window.cpp
│     ├─ map_view_widget.cpp
│     └─ status_panel.cpp
│
└─ test/
   ├─ test_measurement_synchronizer.cpp
   ├─ test_scan_deskewer.cpp
   └─ test_lio_estimator.cpp