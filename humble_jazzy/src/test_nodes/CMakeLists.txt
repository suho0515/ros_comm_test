cmake_minimum_required(VERSION 3.8)
project(test_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(test_msgs REQUIRED)

# C++ 노드 빌드
add_executable(publisher_node src/publisher_node.cpp)
ament_target_dependencies(publisher_node
  rclcpp
  std_msgs
  test_msgs
)

add_executable(subscriber_node src/subscriber_node.cpp)
ament_target_dependencies(subscriber_node
  rclcpp
  std_msgs
  test_msgs
)

# Python 모듈 설치
ament_python_install_package(${PROJECT_NAME})

# 실행 파일 설치
install(TARGETS
  publisher_node
  subscriber_node
  DESTINATION lib/${PROJECT_NAME}
)

# Python 스크립트 설치
install(PROGRAMS
  scripts/test_publisher.py
  scripts/test_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package() 