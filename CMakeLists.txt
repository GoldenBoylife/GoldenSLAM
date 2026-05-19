cmake_minimum_required(VERSION 3.8)
project(golden_slam)
cmake_policy(SET CMP0074 NEW)

# =======================
# Build type
# =======================
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release) # Release or Debug
endif()



# =======================
# C++ settings
# =======================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic -fexceptions)
# 경고 켜기


set(PKG "golden_slam")
set(PKG_NODE ${PKG}_node)


# add_definitions( -DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
message(STATUS "Project: ${PKG}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")



# =======================
# Optional multi-core / OpenMp
# =======================

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
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found")
endif()
# OpenMP : 멀티쓰레딩/병렬 처리용 라이브러리 omp_get_wtime()
# QUIET : 못찾아도 조용히넘겨라. 


# =======================
#  ROS2 dependencies
# =======================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
    # sensor
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

find_package(std_srvs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# =========================
# Third-party libraries
# =========================
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io registration search filters kdtree)
find_package(pcl_conversions REQUIRED)
add_definitions(${PCL_DEFINITIONS})
message(STATUS "PCL libraries: ${PCL_LIBRARIES}")
message(STATUS "PCL include dirs: ${PCL_INCLUDE_DIRS}")
message(STATUS "Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "PCL include dirs: ${PCL_INCLUDE_DIRS}")

set(ROS_DEPS
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
  std_srvs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)


# =======================
# Executable
# =======================
add_executable(${PKG_NODE}
  src/app/main.cpp
  src/app/golden_slam_app.cpp
  src/core/slam_core.cpp
  src/core/preprocess.cpp
  src/core/imu_processor.cpp
  src/platform/ros_bridge.cpp
  
  
)
target_compile_definitions(${PKG_NODE} PRIVATE PCL_NO_PRECOMPILE)

target_include_directories(${PKG_NODE} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  include
  ./
)


ament_target_dependencies(${PKG_NODE}
  ${ROS_DEPS}
  livox_ros_driver2
)

target_link_libraries(${PKG_NODE}
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(${PKG_NODE} OpenMP::OpenMP_CXX)
endif()

# =======================
# Install
# =======================
install(TARGETS ${PKG_NODE}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# =========================
# Ament package
# =========================
ament_package()