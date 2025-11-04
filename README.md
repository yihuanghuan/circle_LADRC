# PX4 LADRC 圆形轨迹控制器

本项目是一个 ROS 2 软件包，旨在使用**线性自抗扰控制器 (LADRC)** 控制 PX4 无人机在 Gazebo 仿真环境中执行高精度的圆形轨迹飞行。

## 项目结构

本项目由两个核心 ROS 2 包组成：

1.  **`ladrc_controller`**:

      * 实现了核心的 LADRC 位置控制器。
      * 控制器由一个**线性扩张状态观测器 (LESO)** 和一个**线性状态误差反馈 (LSEF)** 组成。
      * 此节点 订阅参考轨迹和无人机里程计，计算并向 PX4 发送加速度控制指令。

2.  **`circular_trajectory_planner`**:

      * 实现了一个简单的圆形轨迹生成器。
      * 此节点 负责在启动时自动发送**解锁**和**切换 Offboard 模式**的 PX4 指令。
      * 在进入 Offboard 模式后，它会发布一个平滑过渡（从中心到圆周）并最终循环的圆形轨迹参考点 `/reference_pose`。

## 环境依赖

  * ROS 2 (Humble 或更高版本)
  * PX4 自动驾驶仪 (用于 SITL 仿真)
  * Gazebo 仿真环境
  * `px4_msgs` (ROS 2 消息包)

## 编译

1.  将 `circle_ladrc` 文件夹放置在您的 ROS 2 工作空间 (e.g., `~/ros2_ws/src`)。
2.  编译工作空间：
    ```bash
    cd ~/ros2_ws
    colcon build
    ```

## 如何运行

1.  **启动 PX4 SITL + Gazebo**
    在一个终端中，启动 PX4 仿真（以 iris 机型为例）：

    ```bash
    # (根据您的 PX4 路径修改)
    cd ~/PX4-Autopilot
    make px4_sitl gazebo_iris
    ```

2.  **启动 ROS 2 节点**
    等待 Gazebo 和 PX4 完全启动后，在另一个终端中， sourcing 您的工作空间并运行 `full_system.launch.py`：

    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch circular_trajectory_planner full_system.launch.py
    ```

    启动后，无人机将等待10秒以稳定系统，然后自动执行解锁、切换 Offboard 模式、过渡到圆周并开始执行圆形轨迹。

## 参数配置

### LADRC 控制器 (`ladrc_controller`)

控制器的核心参数位于 `ladrc_controller/config/ladrc_params.yaml`。经过大量迭代调参，以下是针对X轴和Y轴的推荐稳定配置：

```yaml
ladrc_position_controller:
  ros__parameters:
    # ...
    
    # X-axis LADRC parameters
    omega_o_x: 2.0      # 观测器带宽 (wo)
    omega_c_x: 0.8      # 控制器带宽 (wc)
    b0_x: 0.8           # 控制增益估计 (b0)
    
    # Y-axis LADRC parameters
    omega_o_y: 2.0      # 观测器带宽 (wo)
    omega_c_y: 0.8      # 控制器带宽 (wc)
    b0_y: 0.8           # 控制增益估计 (b0)
    
    # Z-axis LADRC parameters (保持默认)
    omega_o_z: 10.0
    omega_c_z: 2.0
    b0_z: 0.9
    
    # ...
```

### 轨迹规划器 (`circular_trajectory_planner`)

轨迹的关键参数（如半径和角速度）在 `circular_trajectory_planner_node.cpp` 中通过 `declare_parameter` 设置了默认值。

  * **`radius`**: 5.0 (米)
  * **`angular_velocity`**: 0.5 (弧度/秒)
  * **`center_z`**: 2.0 (米)
  * **`transition_duration`**: 5.0 (秒, 从中心过渡到圆周的时间)



## 节点架构 (Topics)

  * **`ladrc_position_controller_node`**

      * **[订阅]** `/reference_pose` (`geometry_msgs/msg/PoseStamped`): 接收规划器发来的目标位姿。
      * **[订阅]** `/fmu/out/vehicle_odometry` (`px4_msgs/msg/VehicleOdometry`): 接收 PX4 发来的无人机当前状态（位置在NED坐标系）。
      * **[发布]** `/fmu/in/trajectory_setpoint` (`px4_msgs/msg/TrajectorySetpoint`): 向 PX4 发送计算出的加速度指令（NED坐标系）。
      * **[发布]** `/fmu/in/offboard_control_mode` (`px4_msgs/msg/OffboardControlMode`): 持续保持 Offboard 模式。

  * **`circular_trajectory_planner_node`**

      * **[发布]** `/reference_pose` (`geometry_msgs/msg/PoseStamped`): 向控制器发布目标位姿（ENU坐标系）。
      * **[发布]** `/fmu/in/vehicle_command` (`px4_msgs/msg/VehicleCommand`): 在启动时向 PX4 发送解锁和切换模式指令。

## 许可证

本项目基于 Apache 2.0 许可证。