# 基于LADRC控制器和圆形轨迹规划器的PX4-ROS2仿真

本项目在ROS2/Gazebo仿真环境中，为PX4无人机实现了一个线***扰控制（LADRC）位置控制器，并配合一个圆形轨迹规划器，实现了无人机的自主起飞与轨迹跟踪。

## 功能特性

* **LADRC 位置控制**: 使用 `ladrc_controller` 包替换了PX4内部的PID位置控制器。
* **圆形轨迹规划**: `circular_trajectory_planner` 包生成平滑的圆形轨迹参考点。
* **自主起飞与模式切换**: 无人机启动后将自动执行解锁(Arm)、起飞(Takeoff)并切换到Offboard模式，无需手动干预。
* **修正的PX4接口**: 控制器使用标准 `nav_msgs::msg::Odometry` 话题，并通过速度和加速度指令正确控制无人机。

## 软件包结构

### 1. `ladrc_controller`
一个可重用的LADRC控制器实现。

* `src/ladrc_position_controller_node.cpp`: ROS2节点，订阅参考位姿和里程计信息，计算LADRC控制指令。
* `src/ladrc_core.cpp`, `src/leso.cpp`, `src/lsef.cpp`: LADRC核心算法（控制器、扩张状态观测器、状态误差反馈）。
* `config/ladrc_params.yaml`: LADRC控制器的调优参数。

### 2. `circular_trajectory_planner`
生成圆形轨迹，并管理无人机起飞序列。

* `src/circular_trajectory_planner_node.cpp`: ROS2节点，执行自主起飞，并发布圆形轨迹 `/reference_pose`。
* `config/trajectory_params.yaml`: 轨迹参数（如半径、速度）。
* `launch/full_system.launch.py`: 组合启动文件，同时启动LADRC控制器和轨迹规划器。

## 依赖项

* ROS2 (Humble 或更高版本)
* PX4 Autopilot (已安装 `px4_ros_com` 和 `px4_msgs`)
* Gazebo 仿真环境

## 编译

在修改代码后（尤其是 `circular_trajectory_planner`），请务必重新编译：

```bash
# 进入您的ROS2工作空间
cd ~/ros2_ws
# 编译这两个包
colcon build --packages-select ladrc_controller circular_trajectory_planner
# Source环境
source install/setup.bash