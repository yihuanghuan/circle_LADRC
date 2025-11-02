#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

#include "ladrc_controller/ladrc_core.hpp"

class LADRCPositionControllerNode : public rclcpp::Node
{
public:
  LADRCPositionControllerNode()
    : Node("ladrc_position_controller")
  {
    // Declare parameters
    this->declare_parameter("control_frequency", 50.0);
    this->declare_parameter("omega_o_x", 15.0);
    this->declare_parameter("omega_o_y", 15.0);
    this->declare_parameter("omega_o_z", 15.0);
    this->declare_parameter("omega_c_x", 8.0);
    this->declare_parameter("omega_c_y", 8.0);
    this->declare_parameter("omega_c_z", 8.0);
    this->declare_parameter("b0_x", 1.0);
    this->declare_parameter("b0_y", 1.0);
    this->declare_parameter("b0_z", 1.0);
    this->declare_parameter("max_velocity", 5.0);
    this->declare_parameter("max_acceleration_z", 3.0);

    // Get parameters
    double control_freq = this->get_parameter("control_frequency").as_double();
    dt_ = 1.0 / control_freq;
    max_vel_ = this->get_parameter("max_velocity").as_double();
    max_acc_z_ = this->get_parameter("max_acceleration_z").as_double();

    // Initialize LADRC controllers for x, y, z
    initializeControllers();

    // Subscribers
    reference_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/reference_pose", 10,
      std::bind(&LADRCPositionControllerNode::referenceCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&LADRCPositionControllerNode::odomCallback, this, std::placeholders::_1));

    // Publishers
    offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", 10);
    
    trajectory_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", 10);
    
    vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", 10);

    // Control timer
    auto timer_period = std::chrono::duration<double>(dt_);
    control_timer_ = this->create_wall_timer(
      timer_period,
      std::bind(&LADRCPositionControllerNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "LADRC Position Controller Node initialized");
  }

private:
  void initializeControllers()
  {
    ladrc_controller::LADRCParams params_x, params_y, params_z;
    
    // X-axis controller
    params_x.omega_o = this->get_parameter("omega_o_x").as_double();
    params_x.omega_c = this->get_parameter("omega_c_x").as_double();
    params_x.kp = params_x.omega_c * params_x.omega_c;
    params_x.kd = 2.0 * params_x.omega_c;
    params_x.b0 = this->get_parameter("b0_x").as_double();
    params_x.dt = dt_;
    params_x.max_output = max_vel_;
    params_x.min_output = -max_vel_;
    
    // Y-axis controller
    params_y.omega_o = this->get_parameter("omega_o_y").as_double();
    params_y.omega_c = this->get_parameter("omega_c_y").as_double();
    params_y.kp = params_y.omega_c * params_y.omega_c;
    params_y.kd = 2.0 * params_y.omega_c;
    params_y.b0 = this->get_parameter("b0_y").as_double();
    params_y.dt = dt_;
    params_y.max_output = max_vel_;
    params_y.min_output = -max_vel_;
    
    // Z-axis controller
    params_z.omega_o = this->get_parameter("omega_o_z").as_double();
    params_z.omega_c = this->get_parameter("omega_c_z").as_double();
    params_z.kp = params_z.omega_c * params_z.omega_c;
    params_z.kd = 2.0 * params_z.omega_c;
    params_z.b0 = this->get_parameter("b0_z").as_double();
    params_z.dt = dt_;
    params_z.max_output = max_acc_z_;
    params_z.min_output = -max_acc_z_;
    
    ladrc_x_ = std::make_unique<ladrc_controller::LADRCController>(params_x);
    ladrc_y_ = std::make_unique<ladrc_controller::LADRCController>(params_y);
    ladrc_z_ = std::make_unique<ladrc_controller::LADRCController>(params_z);
  }

  void referenceCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    reference_pose_ = *msg;
    has_reference_ = true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_odom_ = *msg;
    has_odom_ = true;
  }

  void controlLoop()
  {
    if (!has_reference_ || !has_odom_) {
      return;
    }

    // Publish offboard control mode
    publishOffboardControlMode();

    // Get current position
    double x = current_odom_.pose.pose.position.x;
    double y = current_odom_.pose.pose.position.y;
    double z = current_odom_.pose.pose.position.z;

    // Get reference position
    double x_ref = reference_pose_.pose.position.x;
    double y_ref = reference_pose_.pose.position.y;
    double z_ref = reference_pose_.pose.position.z;

    // Calculate control outputs using LADRC
    double vx_cmd = ladrc_x_->update(x_ref, x);
    double vy_cmd = ladrc_y_->update(y_ref, y);
    double az_cmd = ladrc_z_->update(z_ref, z);

    // Publish trajectory setpoint
    publishTrajectorySetpoint(vx_cmd, vy_cmd, az_cmd, z_ref);

    // Log control info
    if (++log_counter_ >= 50) {  // Log every second at 50Hz
      RCLCPP_INFO(this->get_logger(),
        "Ref: [%.2f, %.2f, %.2f] | Pos: [%.2f, %.2f, %.2f] | Cmd: [%.2f, %.2f, %.2f]",
        x_ref, y_ref, z_ref, x, y, z, vx_cmd, vy_cmd, az_cmd);
      log_counter_ = 0;
    }
  }

  void publishOffboardControlMode()
  {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.position = false;
    msg.velocity = true;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    
    offboard_mode_pub_->publish(msg);
  }

  void publishTrajectorySetpoint(double vx, double vy, double az, double z_ref)
  {
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    
    msg.position = {NAN, NAN, static_cast<float>(-z_ref)};  // NED frame
    msg.velocity = {static_cast<float>(vx), static_cast<float>(vy), NAN};
    msg.acceleration = {NAN, NAN, static_cast<float>(-az)};
    msg.yaw = reference_pose_.pose.orientation.z;
    
    trajectory_pub_->publish(msg);
  }

  // Member variables
  std::unique_ptr<ladrc_controller::LADRCController> ladrc_x_;
  std::unique_ptr<ladrc_controller::LADRCController> ladrc_y_;
  std::unique_ptr<ladrc_controller::LADRCController> ladrc_z_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr reference_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  geometry_msgs::msg::PoseStamped reference_pose_;
  nav_msgs::msg::Odometry current_odom_;

  bool has_reference_ = false;
  bool has_odom_ = false;
  double dt_;
  double max_vel_;
  double max_acc_z_;
  int log_counter_ = 0;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LADRCPositionControllerNode>());
  rclcpp::shutdown();
  return 0;
}