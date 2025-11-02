# LADRC Controller and Circle Trajectory Planner for PX4

This project implements a Linear Active Disturbance Rejection Controller (LADRC) for UAV control and a circle trajectory planner for PX4-based drones in ROS2/Gazebo simulation.

## Package Structure

The project consists of two separate ROS2 packages:

### 1. `ladrc_controller` Package
A generic, reusable LADRC controller implementation that can be used for various UAV control applications.

**Files:**
- `include/ladrc_controller/ladrc_controller.hpp` - Main header file with controller classes
- `src/extended_state_observer.cpp` - ESO implementation
- `src/ladrc_controller.cpp` - LADRC controller implementation
- `src/ladrc_controller_node.cpp` - ROS2 node main file
- `launch/ladrc_controller.launch.py` - Launch file for the controller
- `package.xml` - Package manifest
- `CMakeLists.txt` - Build configuration

### 2. `trajectory_planner` Package
Trajectory planning package that generates circle trajectories and interfaces with the LADRC controller.

**Files:**
- `src/circle_trajectory_node.cpp` - Circle trajectory generator node
- `launch/circle_trajectory.launch.py` - Launch file for trajectory planner
- `launch/circle_flight_with_ladrc.launch.py` - Combined launch file
- `config/ladrc_params.yaml` - LADRC tuning parameters
- `config/trajectory_params.yaml` - Trajectory parameters
- `package.xml` - Package manifest
- `CMakeLists.txt` - Build configuration

## Prerequisites

- ROS2 (Foxy or later)
- PX4 Autopilot
- Gazebo simulator
- px4_msgs package

## Building the Packages

1. Place both packages in your ROS2 workspace:
```bash
cd ~/ros2_ws/src
cp -r /path/to/ladrc_controller .
cp -r /path/to/trajectory_planner .
```

2. Build the workspace:
```bash
cd ~/ros2_ws
colcon build --packages-select ladrc_controller trajectory_planner
source install/setup.bash
```

## Usage

### Step 1: Start PX4 SITL with Gazebo
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo
```

### Step 2: Launch MicroXRCEDDS Agent
In a new terminal:
```bash
MicroXRCEAgent udp4 -p 8888
```

### Step 3: Launch the LADRC Controller and Trajectory Planner
In a new terminal:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch trajectory_planner circle_flight_with_ladrc.launch.py
```

Or launch them separately:
```bash
# Terminal 1 - LADRC Controller
ros2 launch ladrc_controller ladrc_controller.launch.py

# Terminal 2 - Trajectory Planner
ros2 launch trajectory_planner circle_trajectory.launch.py
```

## ROS2 Topics

### Published by Trajectory Planner:
- `/ladrc/reference_pose` - Reference position and orientation
- `/ladrc/reference_velocity` - Reference velocities
- `/fmu/in/offboard_control_mode` - Offboard control mode
- `/fmu/in/vehicle_command` - Vehicle commands (arm/disarm)

### Published by LADRC Controller:
- `/fmu/in/trajectory_setpoint` - Control commands to PX4
- `/ladrc/debug_disturbance` - Estimated disturbances (for debugging)

### Subscribed by LADRC Controller:
- `/fmu/out/vehicle_local_position` - Current position from PX4
- `/fmu/out/vehicle_attitude` - Current attitude from PX4
- `/ladrc/reference_pose` - Reference from planner
- `/ladrc/reference_velocity` - Reference velocities from planner

## Parameter Tuning

### LADRC Parameters
Edit `trajectory_planner/config/ladrc_params.yaml`:

Key parameters:
- `position_observer_bandwidth`: ESO bandwidth for X/Y position (10-20 rad/s)
- `position_controller_bandwidth`: Controller bandwidth for X/Y (1-5 rad/s)
- `altitude_observer_bandwidth`: ESO bandwidth for altitude (8-15 rad/s)
- `altitude_controller_bandwidth`: Controller bandwidth for altitude (1-3 rad/s)

Tuning guidelines:
1. Observer bandwidth should be 3-10 times higher than controller bandwidth
2. Start with default values and adjust based on performance
3. Increase observer bandwidth for faster disturbance rejection
4. Increase controller bandwidth for faster tracking (but may cause overshoot)

### Trajectory Parameters
Edit `trajectory_planner/config/trajectory_params.yaml`:

- `takeoff_altitude`: Target altitude in NED frame (negative is up)
- `circle_radius`: Radius of the circular path
- `angular_velocity`: Speed of circular motion (rad/s)

## LADRC Controller Features

The LADRC controller package is designed to be reusable and includes:

1. **Extended State Observer (ESO)**: Estimates system states and total disturbances
2. **Bandwidth Parameterization**: Easy tuning via bandwidth parameters
3. **Multi-axis Control**: Independent controllers for X, Y, Z, and Yaw
4. **Output Saturation**: Configurable velocity limits
5. **Modular Design**: Can be easily integrated into other projects

## Migrating LADRC to Other Projects

To use the LADRC controller in other projects:

1. Copy the `ladrc_controller` package to your workspace
2. Include the header in your code:
   ```cpp
   #include "ladrc_controller/ladrc_controller.hpp"
   ```
3. Create controller instances:
   ```cpp
   ladrc_controller::LADRCAxisController controller;
   controller.initialize(observer_bw, controller_bw, b0, dt);
   ```
4. Use in control loop:
   ```cpp
   double control = controller.compute(reference, reference_dot, measurement);
   ```

## Monitor and Debug

View controller performance:
```bash
# Monitor reference tracking
ros2 topic echo /ladrc/reference_pose

# Monitor control outputs
ros2 topic echo /fmu/in/trajectory_setpoint

# Monitor disturbance estimates
ros2 topic echo /ladrc/debug_disturbance

# Monitor vehicle position
ros2 topic echo /fmu/out/vehicle_local_position
```

## Troubleshooting

1. **Drone doesn't take off**: Check if offboard mode is activated and vehicle is armed
2. **Unstable flight**: Reduce controller bandwidth or increase observer bandwidth
3. **Slow response**: Increase controller bandwidth carefully
4. **High-frequency oscillations**: Reduce observer bandwidth

## References

- Gao, Z. (2006). "Active disturbance rejection control: a paradigm shift in feedback control system design"
- Han, J. (2009). "From PID to active disturbance rejection control"
- PX4 Offboard Control: https://docs.px4.io/main/en/flight_modes/offboard.html
