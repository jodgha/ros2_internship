cmake_minimum_required(VERSION 3.8)
project(vehicle_4axle_8steer)

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  else()
    message(STATUS "ament_lint_auto 未安装，跳过 lint 测试。可通过 apt install ros-${ROS_DISTRO}-ament-lint-common 安装")
  endif()
endif()

# ── 安装 URDF / Launch / RViz 资源 ──────────────────────
install(DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# ── 安装 Python 包 vehicle_4axle (转向计算库) ────────────
# 安装到 lib/${PROJECT_NAME}/vehicle_4axle/，与脚本同目录
# 脚本中通过 `from vehicle_4axle.steer_calc import ...` 导入
install(DIRECTORY vehicle_4axle
  DESTINATION lib/${PROJECT_NAME}
)

# ── 安装可执行脚本 ──────────────────────────────────────
# core/   — 核心节点
# keyboard/ — 键盘遥控
# gb29/   — GB29 方向盘
install(PROGRAMS
  scripts/core/steering_controller.py
  scripts/core/odometry_publisher.py
  scripts/core/vehicle_dynamics.py
  scripts/keyboard/remote_controller.py
  scripts/gb29/gb29_controller.py
  scripts/gb29/gb29_test.py
  scripts/demo_steer.py
  scripts/path_tracker.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── PYTHONPATH 环境钩子 ─────────────────────────────────
# 让 `from vehicle_4axle.steer_calc import ...` 在 source 后直接可用
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/pythonpath.dsv.in"
)

ament_package()
