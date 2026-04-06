cmake_minimum_required(VERSION 3.20)
project(autonomous_robot_localization_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(std_srvs REQUIRED)
pkg_check_modules(libcamera REQUIRED libcamera)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# --- Photographer node ---
add_executable(photographer_node src/photographer.cpp)
target_include_directories(photographer_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${libcamera_INCLUDE_DIRS}
)
ament_target_dependencies(photographer_node rclcpp sensor_msgs)
target_link_libraries(photographer_node
  ${OpenCV_LIBRARIES}
  ${libcamera_LIBRARIES}
)

# --- Visual odometry node ---
add_executable(visual_node src/visual_odometry.cpp)
# 3. CORRECCIÓN: Typo arreglado (PUBLIC_ -> PUBLIC)
target_include_directories(visual_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(visual_node rclcpp sensor_msgs geometry_msgs)
target_link_libraries(visual_node
  ${OpenCV_LIBRARIES}
)

# --- Streaming node ---
add_executable(streaming_node src/streaming_node.cpp)
target_include_directories(streaming_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${libcamera_INCLUDE_DIRS} # 4. CORRECCIÓN: Añadido acceso a libcamera
)
ament_target_dependencies(streaming_node rclcpp sensor_msgs)
target_link_libraries(streaming_node
  ${OpenCV_LIBRARIES}
  ${libcamera_LIBRARIES}      # 4. CORRECCIÓN: Añadido acceso a libcamera
  Threads::Threads            # 4. CORRECCIÓN: Enlace correcto a pthread
)

# --- Mapper node ---
add_executable(mapper_node src/mapper.cpp)
target_include_directories(mapper_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mapper_node rclcpp sensor_msgs geometry_msgs)
target_link_libraries(mapper_node
  ${OpenCV_LIBRARIES}
)

# --- IMU node ---
add_executable(imu_node src/imu_node.cpp src/imu_class.cpp src/ADIS16460_driver.cpp)
target_include_directories(imu_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(imu_node rclcpp sensor_msgs std_srvs)

# Installation
install(TARGETS 
  photographer_node
  visual_node
  streaming_node
  mapper_node
  imu_node
  DESTINATION lib/${PROJECT_NAME}
)

# Launchers
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Exporta el paquete
ament_package()