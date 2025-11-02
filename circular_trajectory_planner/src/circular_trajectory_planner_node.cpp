#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <cmath>
#include <chrono>
#include <atomic>

using namespace std::chrono_literals;

class CircularTrajectoryPlannerNode : public rclcpp::Node
{
public:
  CircularTrajectoryPlannerNode()
    : Node("circular_trajectory_planner"), time_(0.0)
  {
    // 声明参数
    this->declare_parameter("center_x", 0.0);
    this->declare_parameter("center_y", 0.0);
    this->declare_parameter("center_z", 2.0); // 注意：PX4中高度是负值，但这里我们使用ROS ENU坐标系
    this->declare_parameter("radius", 5.0);
    this->declare_parameter("angular_velocity", 0.5);  // rad/s
    this->declare_parameter("publish_rate", 50.0);      // Hz
    this->declare_parameter("visualize", true);

    // 获取参数
    center_x_ = this->get_parameter("center_x").as_double();
    center_y_ = this->get_parameter("center_y").as_double();
    center_z_ = this->get_parameter("center_z").as_double();
    radius_ = this->get_parameter("radius").as_double();
    angular_vel_ = this->get_parameter("angular_velocity").as_double();
    double publish_rate = this->get_parameter("publish_rate").as_double();
    visualize_ = this->get_parameter("visualize").as_bool();

    dt_ = 1.0 / publish_rate;

    // --- 新增：自主起飞状态 ---
    flight_state_ = FlightState::INIT;
    offboard_setpoint_counter_ = 0;

    // --- 新增：PX4 指令发布者 ---
    offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", 10);
    vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", 10);

    // --- 修改：参考位姿发布者 ---
    reference_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/reference_pose", 10);
    
    if (visualize_) {
      path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/planned_path", 10);
      marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/trajectory_marker", 10);
    }

    // --- 修改：分离两个定时器 ---
    // 轨迹发布定时器（高频）
    auto trajectory_timer_period = std::chrono::duration<double>(dt_);
    trajectory_timer_ = this->create_wall_timer(
      trajectory_timer_period,
      std::bind(&CircularTrajectoryPlannerNode::publishTrajectory, this));

    // 状态机与指令定时器（低频）
    auto command_timer_period = std::chrono::milliseconds(100); // 10 Hz
    command_timer_ = this->create_wall_timer(
      command_timer_period,
      std::bind(&CircularTrajectoryPlannerNode::stateMachine, this));


    if (visualize_) {
      // 延迟发布可视化，等待RVIZ启动
      auto viz_timer = this->create_wall_timer(5s, [this]() -> void {
        publishPathVisualization();
        publishCircleMarker();
        // 运行一次后停止
        this->viz_timer_->cancel();
      });
      viz_timer_ = viz_timer;
    }

    RCLCPP_INFO(this->get_logger(), 
      "轨迹规划器已初始化: center=[%.2f, %.2f, %.2f], radius=%.2f, omega=%.2f rad/s",
      center_x_, center_y_, center_z_, radius_, angular_vel_);
    RCLCPP_INFO(this->get_logger(), "等待系统稳定... 将在10秒后开始执行自主起飞序列。");
  }

private:
  // --- 新增：飞行状态机 ---
  enum class FlightState {
    INIT,
    WAITING_FOR_STABLE,
    ARMING,
    TAKEOFF,
    SETTING_OFFBOARD,
    RUNNING_TRAJECTORY
  };
  std::atomic<FlightState> flight_state_;
  std::atomic<uint64_t> offboard_setpoint_counter_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr viz_timer_;

  // --- 新增：指令发布函数 ---
  void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param7 = 0.0)
  {
    px4_msgs::msg::VehicleCommand msg{};
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param7 = param7; // param7 for altitude in takeoff
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_pub_->publish(msg);
  }

  void publishOffboardControlMode()
  {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true; // 我们将发送位置/速度/加速度指令
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_pub_->publish(msg);
  }

  // --- 新增：状态机 ---
  void stateMachine()
  {
    // 持续发布Offboard控制模式，这是PX4进入Offboard模式的先决条件
    publishOffboardControlMode();

    switch (flight_state_.load()) {
      case FlightState::INIT:
        // 等待10秒，让Gazebo和控制器节点稳定
        if (++offboard_setpoint_counter_ * 100 > 10000) { // 100ms * 100 = 10s
          RCLCPP_INFO(this->get_logger(), "系统稳定，开始解锁 (Arming)...");
          publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
          flight_state_ = FlightState::ARMING;
          offboard_setpoint_counter_ = 0;
        }
        break;

      case FlightState::ARMING:
        // 等待2秒
        if (++offboard_setpoint_counter_ * 100 > 2000) {
          RCLCPP_INFO(this->get_logger(), "发送起飞指令 (Takeoff)至 %.2f 米...", center_z_);
          // 注意：PX4的CMD_NAV_TAKEOFF使用NED坐标系下的高度，但此处param7似乎是相对高度
          // 为简单起见，我们使用LADRC控制器期望的ENU高度
          // 我们只发送起飞指令，LADRC控制器将处理悬停
          publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 0.0, 0.0, center_z_);
          flight_state_ = FlightState::TAKEOFF;
          offboard_setpoint_counter_ = 0;
        }
        break;

      case FlightState::TAKEOFF:
        // 等待15秒让无人机起飞并稳定在悬停点
        if (++offboard_setpoint_counter_ * 100 > 15000) {
          RCLCPP_INFO(this->get_logger(), "切换到 Offboard 模式...");
          publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0); // 6.0 = MAV_MODE_OFFBOARD
          flight_state_ = FlightState::SETTING_OFFBOARD;
          offboard_setpoint_counter_ = 0;
        }
        break;

      case FlightState::SETTING_OFFBOARD:
        // 等待1秒确保模式切换成功
         if (++offboard_setpoint_counter_ * 100 > 1000) {
          RCLCPP_INFO(this->get_logger(), "Offboard 模式已激活。开始执行圆形轨迹。");
          flight_state_ = FlightState::RUNNING_TRAJECTORY;
         }
        break;
      
      case FlightState::RUNNING_TRAJECTORY:
        // 状态机任务完成，轨迹将在 publishTrajectory() 中运行
        // 我们可以停止这个定时器以节省资源
        command_timer_->cancel();
        break;
    }
  }


  // --- 修改：轨迹发布函数 ---
  void publishTrajectory()
  {
    geometry_msgs::msg::PoseStamped reference_pose;
    reference_pose.header.stamp = this->get_clock()->now();
    reference_pose.header.frame_id = "map"; // 假设使用 ENU "map" 坐标系

    double theta = 0.0;
    double current_yaw = 0.0;

    if (flight_state_.load() == FlightState::RUNNING_TRAJECTORY) {
      // 只有在进入轨迹运行状态后才开始增加时间
      theta = angular_vel_ * time_;
      time_ += dt_;
      current_yaw = theta + M_PI / 2.0; // 偏航角切向圆周
      
      reference_pose.pose.position.x = center_x_ + radius_ * std::cos(theta);
      reference_pose.pose.position.y = center_y_ + radius_ * std::sin(theta);
    } else {
      // 在起飞和切换模式期间，保持在起始点悬停
      reference_pose.pose.position.x = center_x_;
      reference_pose.pose.position.y = center_y_;
      current_yaw = M_PI / 2.0; // 保持朝向（例如，Y轴正方向）
    }
    
    reference_pose.pose.position.z = center_z_; // 高度始终由参数控制

    // Convert yaw to quaternion (simplified for yaw-only rotation)
    reference_pose.pose.orientation.x = 0.0;
    reference_pose.pose.orientation.y = 0.0;
    reference_pose.pose.orientation.z = std::sin(current_yaw / 2.0);
    reference_pose.pose.orientation.w = std::cos(current_yaw / 2.0);

    reference_pub_->publish(reference_pose);
  }

  // --- 可视化函数 (无修改) ---
  void publishPathVisualization()
  {
    nav_msgs::msg::Path path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "map";
    int num_points = 100;
    for (int i = 0; i <= num_points; ++i) {
      double theta = 2.0 * M_PI * i / num_points;
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = center_x_ + radius_ * std::cos(theta);
      pose.pose.position.y = center_y_ + radius_ * std::sin(theta);
      pose.pose.position.z = center_z_;
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);
  }

  void publishCircleMarker()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "circular_trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    int num_points = 100;
    for (int i = 0; i <= num_points; ++i) {
      double theta = 2.0 * M_PI * i / num_points;
      geometry_msgs::msg::Point point;
      point.x = center_x_ + radius_ * std::cos(theta);
      point.y = center_y_ + radius_ * std::sin(theta);
      point.z = center_z_;
      marker.points.push_back(point);
    }
    marker_pub_->publish(marker);
  }

  // --- 成员变量 ---
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr reference_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr trajectory_timer_;

  // --- 新增的成员变量 ---
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

  double center_x_, center_y_, center_z_;
  double radius_;
  double angular_vel_;
  double dt_;
  double time_;
  bool visualize_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CircularTrajectoryPlannerNode>());
  rclcpp::shutdown();
  return 0;
}