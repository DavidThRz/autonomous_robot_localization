cmake_minimum_required(VERSION 3.20)
project(location_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)

# Incluye el directorio donde está httplib.h
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include 
    /usr/local/include
)

# Añade el ejecutable
add_executable(visual_node src/visual_odometry.cpp)

# Vincula las bibliotecas necesarias
target_link_libraries(visual_node PUBLIC
  rclcpp::rclcpp
  ${OpenCV_LIBRARIES}
)

# Instala el ejecutable
install(TARGETS visual_node
  DESTINATION lib/${PROJECT_NAME})

# Exporta el paquete
ament_package()