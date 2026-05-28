cmake_minimum_required(VERSION 3.8)
project(fast_lio)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

 ADD_COMPILE_OPTIONS(-std=c++17) 
 set(CMAKE_CXX_FLAGS "-std=c++17 -O3") 
 # -O3 : 실행속도 극대화
  
 add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\") 
  
 set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions") 
 set(CMAKE_CXX_STANDARD 17) 
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")

# 현재 빌드 중인 pc의 cpu 아키텍쳐 확인.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount) # import function 
  ProcessorCount(N) # detect core count
  message("Processer number:  ${N}")

  ## if core number is upper 4, 
  if(N GREATER 4)
    add_definitions(-DMP_EN) # 멀티 코어 turn on
    add_definitions(-DMP_PROC_NUM=3) # use 3 core
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "(arm)|(ARM)|(aarch64)|(AARCH64)")
  include(ProcessorCount)
  ProcessorCount(N) # count jetson cpu core 
  message("ARM Processor number: ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN) 
    add_definitions(-DMP_PROC_NUM=4) 
    message("core for MP: 3")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

# ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  pcl_ros
  pcl_conversions
  livox_ros_driver2
)

# Thirdparty libraries
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

message(Eigen: ${EIGEN3_INCLUDE_DIR})
message(STATUS "PCL: ${PCL_INCLUDE_DIRS}")


set(msg_files
  "msg/Pose6D.msg"
)

## convert ".msg" to ".hpp"
rosidl_generate_interfaces(${PROJECT_NAME} 
  ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)
# Pose6D.msg의 값을 배포할때 다른 패키지들도 이 메시지 읽도록 배포 허용
# Allow other package to use this ".msg" when distributing the package.

add_executable(fastlio_mapping src/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp)
# 3개를 각각 빌드하고 하나로 합쳐서 fastlio_mapping이라는 하나의 실행파일을 마듬. 

# 컴파일러에게 "코드에서 #include하는 파일들이 어디있는지"를 알려줌.
target_include_directories(fastlio_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>   #현재 위치의 include 살펴봐
  $<INSTALL_INTERFACE:include> #install쪽 include봐
  ${PCL_INCLUDE_DIRS}   #PCL 라이브러리의 헤더도 추가 
)

target_link_libraries(fastlio_mapping ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} Eigen3::Eigen)
# PCL_LIBRARES : point cloude쪽
# PYTHON_: 그래프 출력 
# Eigen3::Eigen : 칼만 필터 연산에 필수적인 고성능 행렬 계산기.
target_include_directories(fastlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})

list(APPEND EOL_LIST "foxy" "galactic" "eloquent" "dashing" "crystal")

#EOL : End of Life 지원 종료된 구 버전은  실행파일 연결 방식이 바귐. 
if($ENV{ROS_DISTRO} IN_LIST EOL_LIST)
  # Custommsg to support foxy & galactic
  rosidl_target_interfaces(fastlio_mapping
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
    # 구버전은 rosidl_target_interfaces 이 함수로 처리
else()
  rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
    # rosidl_get_typesupport_target
  target_link_libraries(fastlio_mapping ${cpp_typesupport_target})
endif()

ament_target_dependencies(fastlio_mapping ${dependencies})
# 아까 위에서 set했던 dependencies등 ROS2 표준 패키지들을  fastlio_mapping에 한번에 연결함. 
# target_link_libraries보다 ROS2에 최적화된 명령어,

# ---------------- Install --------------- #
install(TARGETS fastlio_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
#패키지 설정 끝