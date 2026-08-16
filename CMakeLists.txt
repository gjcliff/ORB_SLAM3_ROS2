cmake_minimum_required(VERSION 3.16)
project(orb_slam3_ros2)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Type of build." FORCE)
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(ENV{PROJECT_PATH} "${PROJECT_SOURCE_DIR}")
add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")
add_definitions(${PCL_DEFINITIONS})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(realsense2_description REQUIRED)
find_package(xacro REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.14 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(yaml-cpp REQUIRED)
find_package(nav2_map_server REQUIRED)

set(ORB_SLAM3_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ORB_SLAM3)
find_package(ORB_SLAM3 REQUIRED CONFIG
    PATHS ${ORB_SLAM3_ROOT_DIR}/build
    NO_DEFAULT_PATH
)
find_package(realsense2)

if(TARGET ORB_SLAM3::ORB_SLAM3)
    message("Found ORB_SLAM3::ORB_SLAM3")
else()
    message("Could not find ORB_SLAM3 library")
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  nav_msgs
  cv_bridge
  realsense2_description
  xacro
  pcl_ros
  pcl_conversions
  Pangolin
  # imu_filter_madgwick
  yaml-cpp
  nav2_map_server
)

add_executable(imu_mono_node_cpp
  src/imu_mono_realsense.cpp
)

add_executable(orb_camera_info_node
  src/orb_camera_info.cpp
)

add_executable(visualize_node
  src/visualize.cpp
)

add_executable(orb_alt
  src/orb_alt.cpp
)

ament_target_dependencies(imu_mono_node_cpp
  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_target_dependencies(orb_camera_info_node
  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_target_dependencies(visualize_node
  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_target_dependencies(orb_alt
  PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

include_directories(
    ${ORB_SLAM3_ROOT_DIR}
    ${ORB_SLAM3_ROOT_DIR}/include
    ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
    ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
    ${PCL_INCLUDE_DIRS}
    ${realsense2_INCLUDE_DIR}
)

target_include_directories(imu_mono_node_cpp PUBLIC
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(imu_mono_node_cpp PUBLIC ORB_SLAM3::ORB_SLAM3 ${PCL_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)
target_link_libraries(orb_camera_info_node PUBLIC yaml-cpp ${PCL_LIBRARIES})
target_link_libraries(orb_alt PUBLIC ORB_SLAM3::ORB_SLAM3 ${PCL_LIBRARIES} ${OpenCV_LIBS} ${realsense2_LIBRARY} yaml-cpp)

install(TARGETS imu_mono_node_cpp orb_camera_info_node visualize_node orb_alt
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY ${ORB_SLAM3_ROOT_DIR}/include
        DESTINATION include
)

install(FILES ${ORB_SLAM3_ROOT_DIR}/lib/libORB_SLAM3.so
        DESTINATION lib
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/ORB_SLAM3/Thirdparty/g2o/lib/libg2o.so
        DESTINATION lib
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2.so
        DESTINATION lib
)

# install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY clouds/
  DESTINATION share/${PROJECT_NAME}/clouds
)

install(
  DIRECTORY database_outputs/
  DESTINATION share/${PROJECT_NAME}/database_outputs
)

install(
    DIRECTORY occupancy_grids/
    DESTINATION share/${PROJECT_NAME}/occupancy_grids
)

# install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
