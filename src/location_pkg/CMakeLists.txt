cmake_minimum_required(VERSION 3.20)
project(location_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)

# Include libraries
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include 
    /usr/local/include
    /usr/include/libcamera
)

# Photographer node
add_executable(photographer_node src/photographer.cpp)
target_link_libraries(photographer_node PUBLIC
  rclcpp::rclcpp
  ${OpenCV_LIBRARIES}
  camera
  camera-base
)
install(TARGETS photographer_node
  DESTINATION lib/${PROJECT_NAME})

# Visual odometry node
add_executable(visual_node src/visual_odometry.cpp)
target_link_libraries(visual_node PUBLIC
  rclcpp::rclcpp
  ${OpenCV_LIBRARIES}
)
install(TARGETS visual_node
  DESTINATION lib/${PROJECT_NAME})

# Exporta el paquete
ament_package()