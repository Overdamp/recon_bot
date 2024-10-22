cmake_minimum_required(VERSION 3.8)
project(recon_bot_aruco_pose_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################ FIND DEPENDENCIES  #################
find_package(ament_cmake REQUIRED)		# cpp (optional)
find_package(rclcpp REQUIRED)			# cpp (optional)
find_package(ament_cmake_python REQUIRED)   	# python (optional)
find_package(rclpy REQUIRED)                	# python (optional)
######################################################

# Install Python Module
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS

scripts/aruco_marker.py

DESTINATION lib/${PROJECT_NAME}
)

################ INSTALL LAUNCH, ETC #################
install(DIRECTORY
  config
  launch
  scripts
  rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
