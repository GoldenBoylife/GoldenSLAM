
# GoldenSLAM V1 구조 설계
## 목표
- core는 ROS/QT 비의존적이게
- platform/ros는 ROS2 입출력, 타이머, 서비스 담당
- ui는 QT 기반 시각화 제어
- app은 전체 조립과 실행 담당
