cmake_minimum_required(VERSION 3.8)
project(recon_bot_mecanum_control)

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

scripts/recon_bot_joy_controller.py
scripts/MobiControl.py
scripts/mobility_node.py
scripts/base_joy.py
scripts/test_motor.py
scripts/mecanum_odometry.py
scripts/read_motor.py

scripts/a_joy_cmd_vel.py  
scripts/a_motor_write.py
scripts/a_motor_read.py   
scripts/a_odometry_pub.py  
scripts/a_square_drive_test.py
scripts/a_motor_read_write.py

DESTINATION lib/${PROJECT_NAME}
)

################ INSTALL LAUNCH, ETC #################
install(DIRECTORY
  launch
  scripts
  DESTINATION share/${PROJECT_NAME})

ament_package()
