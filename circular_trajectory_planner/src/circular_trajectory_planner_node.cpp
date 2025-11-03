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
    this->declare_parameter("transition_duration", 5.0); // <-- [新增] 过渡时间

    // 获取参数
    center_x_ = this->get_parameter("center_x").as_double();
    center_y_ = this->get_parameter("center_y").as_double();
    center_z_ = this->get_parameter("center_z").as_double();
    radius_ = this->get_parameter("radius").as_double();
    angular_vel_ = this->get_parameter("angular_velocity").as_double();
    double publish_rate = this->get_parameter("publish_rate").as_double();
    visualize_ = this->get_parameter("visualize").as_bool();
    transition_duration_ = this->get_parameter("transition_duration").as_double(); // <-- [新增]

    dt_ = 1.0 / publish_rate;

    // --- 自主起飞状态 ---
    flight_state_ = FlightState::INIT;
    offboard_setpoint_counter_ = 0;
    transition_start_time_ = 0.0; // <-- [新增]

    // --- PX4 指令发布者 ---
    vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", 10);

    // --- 参考位姿发布者 ---
    reference_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/reference_pose", 10);
    
    if (visualize_) {
      path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/planned_path", 10);
      marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/trajectory_marker", 10);
    }

    // --- 分离两个定时器 ---
    auto trajectory_timer_period = std::chrono::duration<double>(dt_);
    trajectory_timer_ = this->create_wall_timer(
      trajectory_timer_period,
      std::bind(&CircularTrajectoryPlannerNode::publishTrajectory, this));

    auto command_timer_period = std::chrono::milliseconds(100); // 10 Hz
    command_timer_ = this->create_wall_timer(
      command_timer_period,
      std::bind(&CircularTrajectoryPlannerNode::stateMachine, this));


    if (visualize_) {
      auto viz_timer = this->create_wall_timer(5s, [this]() -> void {
        publishPathVisualization();
        publishCircleMarker();
        this->viz_timer_->cancel();
      });
      viz_timer_ = viz_timer;
    }

    RCLCPP_INFO(this->get_logger(), 
      "轨迹规划器已初始化: center=[%.2f, %.2f, %.2f], radius=%.2f, omega=%.2f rad/s",
      center_x_, center_y_, center_z_, radius_, angular_vel_);
    RCLCPP_INFO(this->get_logger(), "过渡时间: %.2f 秒", transition_duration_);
    RCLCPP_INFO(this->get_logger(), "等待系统稳定... 将在10秒后开始执行自主起飞序列。");
  }

private:
  // --- 飞行状态机 (已修改) ---
  enum class FlightState {
    INIT,
    ARMING,
    SETTING_OFFBOARD,
    TRANSITION_TO_CIRCLE, // <-- [新增] 过渡状态
    RUNNING_TRAJECTORY
  };
  std::atomic<FlightState> flight_state_;
  std::atomic<uint64_t> offboard_setpoint_counter_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr viz_timer_;

  void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param7 = 0.0)
  {
    px4_msgs::msg::VehicleCommand msg{};
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param7 = param7;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_pub_->publish(msg);
  }

  void stateMachine()
  {
    switch (flight_state_.load()) {
      case FlightState::INIT:
        if (++offboard_setpoint_counter_ * 100 > 10000) { 
          RCLCPP_INFO(this->get_logger(), "系统稳定，开始解锁 (Arming)...");
          publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
          flight_state_ = FlightState::ARMING;
          offboard_setpoint_counter_ = 0;
        }
        break;

      case FlightState::ARMING:
        if (++offboard_setpoint_counter_ * 100 > 2000) {
          RCLCPP_INFO(this->get_logger(), "解锁成功。切换到 Offboard 模式...");
          publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0); 
          flight_state_ = FlightState::SETTING_OFFBOARD;
          offboard_setpoint_counter_ = 0;
        }
        break;

      case FlightState::SETTING_OFFBOARD:
         if (++offboard_setpoint_counter_ * 100 > 1000) {
          RCLCPP_INFO(this->get_logger(), "Offboard 模式已激活。开始从中心过渡到圆周...");
          flight_state_ = FlightState::TRANSITION_TO_CIRCLE; // <-- [修改] 切换到过渡状态
          transition_start_time_ = this->get_clock()->now().seconds(); // <-- [新增] 记录过渡开始时间
          command_timer_->cancel(); // <-- [修改] 状态机完成任务，停止
         }
        break;
      
      case FlightState::TRANSITION_TO_CIRCLE: // <-- [修改] 状态机不再需要处理这些
      case FlightState::RUNNING_TRAJECTORY:
        command_timer_->cancel();
        break;
    }
  }

  // --- [核心修改] ---
  void publishTrajectory()
  {
    geometry_msgs::msg::PoseStamped reference_pose;
    reference_pose.header.stamp = this->get_clock()->now();
    reference_pose.header.frame_id = "map"; 

    double theta = 0.0;
    double current_yaw = 0.0;
    double current_radius = 0.0; // <-- [新增] 动态半径

    FlightState current_state = flight_state_.load();
    double current_time_seconds = this->get_clock()->now().seconds();

    if (current_state == FlightState::TRANSITION_TO_CIRCLE) {
      double time_elapsed = current_time_seconds - transition_start_time_;

      if (time_elapsed < transition_duration_) {
        // --- 正在过渡 ---
        // 半径从 0 线性增加到 radius_
        current_radius = radius_ * (time_elapsed / transition_duration_);
        theta = 0.0; // 在过渡期间，我们沿着X轴正向移动
        current_yaw = M_PI / 2.0; // 保持朝向Y轴正向 (或圆的切线方向)
      } else {
        // --- 过渡完成 ---
        RCLCPP_INFO_ONCE(this->get_logger(), "过渡完成。开始执行圆形轨迹。");
        flight_state_ = FlightState::RUNNING_TRAJECTORY;
        time_ = 0.0; // 重置圆形轨迹的计时器
        
        current_radius = radius_;
        theta = 0.0;
        current_yaw = M_PI / 2.0;
      }
    }
    else if (current_state == FlightState::RUNNING_TRAJECTORY) {
      // --- 正在运行圆形轨迹 ---
      theta = angular_vel_ * time_;
      time_ += dt_;
      current_yaw = theta + M_PI / 2.0; // 偏航角始终指向圆的切线方向
      current_radius = radius_;
    } 
    else {
      // --- INIT, ARMING, SETTING_OFFBOARD 状态 ---
      // 保持在圆心 (悬停点)
      current_radius = 0.0; 
      theta = 0.0;
      current_yaw = M_PI / 2.0; 
    }
    
    // --- 根据计算出的动态半径和角度设置目标点 ---
    reference_pose.pose.position.x = center_x_ + current_radius * std::cos(theta);
    reference_pose.pose.position.y = center_y_ + current_radius * std::sin(theta);
    reference_pose.pose.position.z = center_z_; 

    // 设置姿态
    reference_pose.pose.orientation.x = 0.0;
    reference_pose.pose.orientation.y = 0.0;
    reference_pose.pose.orientation.z = std::sin(current_yaw / 2.0);
    reference_pose.pose.orientation.w = std::cos(current_yaw / 2.0);

    reference_pub_->publish(reference_pose);
  }
  // --- [核心修改结束] ---

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

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr reference_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr trajectory_timer_;

  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

  double center_x_, center_y_, center_z_;
  double radius_;
  double angular_vel_;
  double dt_;
  double time_;
  bool visualize_;

  // --- [新增成员变量] ---
  double transition_duration_; // 过渡时间 (秒)
  double transition_start_time_; // 过渡开始的绝对时间 (秒)
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CircularTrajectoryPlannerNode>());
  rclcpp::shutdown();
  return 0;
}