cmake_minimum_required(VERSION 3.22)  
project(pendulum)  


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies  
find_package(ament_cmake REQUIRED)  
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(control_msgs REQUIRED)
find_package(Eigen3 REQUIRED)  
find_package(OsqpEigen REQUIRED) 
find_package(rclcpp REQUIRED)  

# Include directories  
include_directories(  
  include  
)  

# Create a library  
add_library(controller_libs  
  src/controller_lqr.cpp  
  src/controller_mpc.cpp  
)  

# Link libraries  z
target_link_libraries(controller_libs  
  OsqpEigen::OsqpEigen 
)  

# Create an executable  
add_executable(control src/control.cpp)  

# Link the executable with libraries  
target_link_libraries(control  
  controller_libs  
  Eigen3::Eigen  
  OsqpEigen::OsqpEigen
)  
ament_target_dependencies(controller_libs
  rclcpp
  geometry_msgs
  # control_msgs
  sensor_msgs)

# Install targets  
install(TARGETS  
  control  
  controller_libs  
  DESTINATION lib/${PROJECT_NAME}  
)  

# Install include directories  
install(DIRECTORY include/  
  DESTINATION include  
)  


install(DIRECTORY
  launch
  world
  urdf
  script
  DESTINATION share/${PROJECT_NAME}/)

# Ament package  
ament_package()