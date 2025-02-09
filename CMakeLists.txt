cmake_minimum_required(VERSION 3.5)
project(go1_republisher)

# add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(ament_cmake REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# find_package(camera_info_manager_py REQUIRED)
# find_package(cv_bridge REQUIRED)
# find_package(image_transport REQUIRED)
# find_package(OpenCV 4 REQUIRED)

if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
	set(ARCH amd64)
  else()
	set(ARCH arm64)
  endif()
link_directories(${CMAKE_SOURCE_DIR}/../unitree_legged_sdk/lib/cpp/${ARCH})
# link_directories(${CMAKE_SOURCE_DIR}/../UnitreecameraSDK/lib/${ARCH})

# set(EXTRA_LIBS -pthread libunitree_legged_sdk.a libunitree_camera.a libsystemlog.a libtstc_V4L2_xu_camera.a udev)
set(EXTRA_LIBS -pthread libunitree_legged_sdk.a)

include_directories(
	include
	# ${catkin_INCLUDE_DIRS}
	# ${OpenCV_INCLUDE_DIRS}
	${CMAKE_SOURCE_DIR}/../unitree_legged_sdk/include
	# ${CMAKE_SOURCE_DIR}/../UnitreecameraSDK/include
)

add_executable(imu_odom src/imu_odom.cpp)
target_link_libraries(imu_odom ${EXTRA_LIBS})
ament_target_dependencies(imu_odom tf2_ros rclpy rclcpp std_msgs nav_msgs sensor_msgs)

# add_executable(camera src/camera.cpp)
# target_link_libraries(camera ${EXTRA_LIBS} ${OpenCV_LIBRARIES})
# add_dependencies(camera ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(TARGETS imu_odom
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()