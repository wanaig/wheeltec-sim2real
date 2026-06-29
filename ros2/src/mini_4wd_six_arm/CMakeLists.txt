cmake_minimum_required(VERSION 3.8)
project(mini_4wd_six_arm)

find_package(ament_cmake REQUIRED)

# 安装 URDF / meshes / launch / config 到 share 目录, 供 robot_state_publisher 与 launch 使用
install(DIRECTORY urdf meshes launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
