cmake_minimum_required(VERSION 3.20)
project(location_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(JPEG REQUIRED)
find_package(OpenCV REQUIRED)


# Ejecutable
add_executable(visual_node src/visual_odometry.cpp)
# target_include_directories(visual_node PRIVATE ${JPEG_INCLUDE_DIR})
target_include_directories(visual_node PRIVATE)
# target_link_libraries(visual_node ${JPEG_LIBRARIES} rclcpp::rclcpp)
target_link_libraries(visual_node ${OpenCV_LIBS} rclcpp::rclcpp)


# Instalación
install(TARGETS visual_node
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
