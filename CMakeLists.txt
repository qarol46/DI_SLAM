cmake_minimum_required(VERSION 3.8)
project(rgbd_imu_slam)

# Используем C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Поиск зависимостей
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system thread)

# Включаем директории
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
)

# Создаем исполняемый файл
add_executable(imu_preintegration_node src/imuPreintegrations.cpp)

# Линкуем библиотеки
ament_target_dependencies(imu_preintegration_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(imu_preintegration_node
  ${GTSAM_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Установка
install(TARGETS
  imu_preintegration_node
  DESTINATION lib/${PROJECT_NAME}
)

# Установка launch файлов (если создадите)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Установка config файлов
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()