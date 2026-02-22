cmake_minimum_required(VERSION 3.8)
project(sensor_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依存パッケージの検索
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# msgファイルの登録
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorData.msg"
  "msg/SensorData2.msg"
)

ament_package()