cmake_minimum_required(VERSION 3.8)
project(br_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf worlds launch config maps
  DESTINATION share/${PROJECT_NAME}
)

# 工具脚本：ros2 run br_sim <脚本名>.py
#   test_nav.py          Nav2 端到端验证（发初始位姿 + 两个导航目标）
#   check_map.py         地图自检（围墙闭合 / 走廊可达 / 像素统计）
#   make_map_preview.py  地图 PNG 预览（README 配图，需要 python3-pil）
install(
  PROGRAMS
    scripts/test_nav.py
    scripts/check_map.py
    scripts/make_map_preview.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
