/*
 * [Gemini 最终修正 v3]
 * 1. 坐标系和LESO已在 v2 中修复。
 * 2. 根本问题：LADRC (LESO+LSEF) 是一个二阶控制器，输出应为“加速度”。
 * 但原代码将 X/Y 轴的输出当作 "速度" 发送，Z 轴输出当作 "加速度" 发送。
 * 这种混合模式导致 X/Y 轴模型失配，引发不稳定。
 * 3. 修正：
 * a. 将所有轴 (X, Y, Z) 的 LADRC 输出统一视为“加速度”。
 * b. 在 initializeControllers 中，将 X/Y 的 output_limit 也设为加速度限制。
 * c. 在 publishTrajectorySetpoint 中，只发送 acceleration 指令，
 * 停止发送 velocity 指令 (设为 NAN)。
 */

 #include <rclcpp/rclcpp.hpp>
 #include <geometry_msgs/msg/pose_stamped.hpp>
 #include <geometry_msgs/msg/twist_stamped.hpp>
 #include <px4_msgs/msg/vehicle_odometry.hpp> 
 
 #include <px4_msgs/msg/offboard_control_mode.hpp>
 #include <px4_msgs/msg/trajectory_setpoint.hpp>
 #include <px4_msgs/msg/vehicle_command.hpp>
 #include <px4_msgs/msg/vehicle_control_mode.hpp>
 
 #include "ladrc_controller/ladrc_core.hpp"
 #include <cmath> 
 
 class LADRCPositionControllerNode : public rclcpp::Node
 {
 public:
   LADRCPositionControllerNode()
     : Node("ladrc_position_controller")
   {
     // ... (参数声明部分保持不变) ...
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
     this->declare_parameter("max_velocity", 5.0); // 此参数不再用于控制器限制
     this->declare_parameter("max_acceleration_z", 3.0);
 
     // Get parameters
     double control_freq = this->get_parameter("control_frequency").as_double();
     dt_ = 1.0 / control_freq;
     // max_vel_ = this->get_parameter("max_velocity").as_double(); // [修正] 不再使用
     max_acc_ = this->get_parameter("max_acceleration_z").as_double(); // [修正] 统一使用加速度限制
 
     // Initialize LADRC controllers for x, y, z
     initializeControllers();
 
     // Subscribers
     reference_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
       "/reference_pose", rclcpp::QoS(10),
       std::bind(&LADRCPositionControllerNode::referenceCallback, this, std::placeholders::_1));
 
     odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>( 
       "/fmu/out/vehicle_odometry", 
       rclcpp::SensorDataQoS(), 
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
 
     RCLCPP_INFO(this->get_logger(), "LADRC Position Controller Node initialized (已统一为加速度控制器)");
     RCLCPP_INFO(this->get_logger(), "正在等待 /reference_pose 和 /fmu/out/vehicle_odometry 消息...");
   }
 
 private:
   // [修正] X/Y/Z 均使用统一的加速度限制
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
     params_x.max_output = max_acc_;    // <-- [修正]
     params_x.min_output = -max_acc_;   // <-- [修正]
     
     // Y-axis controller
     params_y.omega_o = this->get_parameter("omega_o_y").as_double();
     params_y.omega_c = this->get_parameter("omega_c_y").as_double();
     params_y.kp = params_y.omega_c * params_y.omega_c;
     params_y.kd = 2.0 * params_y.omega_c;
     params_y.b0 = this->get_parameter("b0_y").as_double();
     params_y.dt = dt_;
     params_y.max_output = max_acc_;    // <-- [修正]
     params_y.min_output = -max_acc_;   // <-- [修正]
     
     // Z-axis controller
     params_z.omega_o = this->get_parameter("omega_o_z").as_double();
     params_z.omega_c = this->get_parameter("omega_c_z").as_double();
     params_z.kp = params_z.omega_c * params_z.omega_c;
     params_z.kd = 2.0 * params_z.omega_c;
     params_z.b0 = this->get_parameter("b0_z").as_double();
     params_z.dt = dt_;
     params_z.max_output = max_acc_;    // (保持不变)
     params_z.min_output = -max_acc_;   // (保持不变)
     
     ladrc_x_ = std::make_unique<ladrc_controller::LADRCController>(params_x);
     ladrc_y_ = std::make_unique<ladrc_controller::LADRCController>(params_y);
     ladrc_z_ = std::make_unique<ladrc_controller::LADRCController>(params_z);
   }
 
 
   void referenceCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
   {
     RCLCPP_INFO_ONCE(this->get_logger(), "DEBUG: === 成功接收到 /reference_pose 消息! ===");
     reference_pose_ = *msg;
     has_reference_ = true;
   }
 
   void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
   {
     RCLCPP_INFO_ONCE(this->get_logger(), "DEBUG: === 成功接收到 /fmu/out/vehicle_odometry 消息! ===");
     current_odom_ = *msg;
     has_odom_ = true;
   }
 
   void controlLoop()
   {
     if (!has_reference_ || !has_odom_) {
       if (!has_reference_) {
           RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
             "DEBUG: controlLoop 卡住, 正在等待 /reference_pose... (has_reference_ is false)");
       }
       if (!has_odom_) {
           RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
             "DEBUG: controlLoop 卡住, 正在等待 /fmu/out/vehicle_odometry... (has_odom_ is false)");
       }
       return;
     }
 
     // Publish offboard control mode
     publishOffboardControlMode();
 
     // [正确] 从 PX4(NED) 提取位置并转换为 ROS(ENU)
     // ENU.x (East)  = NED.y (position[1])
     // ENU.y (North) = NED.x (position[0])
     // ENU.z (Up)    = -NED.z (position[2])
     double x = current_odom_.position[1];
     double y = current_odom_.position[0];
     double z = -current_odom_.position[2];
 
     // Get reference position (已经在 ENU 中)
     double x_ref = reference_pose_.pose.position.x;
     double y_ref = reference_pose_.pose.position.y;
     double z_ref = reference_pose_.pose.position.z;
 
     // 此处及之后的所有计算都在 ENU 坐标系中进行
 
     // [修正] Calculate control outputs using LADRC (输出统一为加速度)
     double ax_cmd = ladrc_x_->update(x_ref, x);
     double ay_cmd = ladrc_y_->update(y_ref, y);
     double az_cmd = ladrc_z_->update(z_ref, z);
 
     // ... (Yaw 计算保持不变) ...
     double qz = reference_pose_.pose.orientation.z;
     double qw = reference_pose_.pose.orientation.w;
     double yaw_enu = 2.0 * std::atan2(qz, qw); 
     
     double yaw_ned = -yaw_enu + M_PI / 2.0;
 
     // [修正] 传递加速度指令
     publishTrajectorySetpoint(ax_cmd, ay_cmd, az_cmd, yaw_ned);
 
     // Log control info
     if (++log_counter_ >= 50) {  // Log every second at 50Hz
       RCLCPP_INFO(this->get_logger(),
         "Ref: [%.2f, %.2f, %.2f] | Pos: [%.2f, %.2f, %.2f] | Cmd(A): [%.2f, %.2f, %.2f]",
         x_ref, y_ref, z_ref, x, y, z, ax_cmd, ay_cmd, az_cmd);
       log_counter_ = 0;
     }
   }
 
   // [修正] OffboardControlMode 保持不变，PX4 允许同时设置 T/V/A
   void publishOffboardControlMode()
   {
     px4_msgs::msg::OffboardControlMode msg{};
     msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
     msg.position = false;
     msg.velocity = true;
     msg.acceleration = true; 
     msg.attitude = false;
     msg.body_rate = false;
     
     offboard_mode_pub_->publish(msg);
   }
 
   // [修正] 将 ax, ay, az (ENU) 转换为 PX4 (NED) 加速度指令
   void publishTrajectorySetpoint(double ax_enu, double ay_enu, double az_enu, double yaw_ref)
   {
     // ax_enu, ay_enu, az_enu 是来自LADRC的 ENU 加速度指令
     px4_msgs::msg::TrajectorySetpoint msg{};
     msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
     
     msg.position = {NAN, NAN, NAN}; 
     msg.velocity = {NAN, NAN, NAN}; // <-- [修正] 停止发送速度指令
 
     // [修正] 将 ENU 加速度指令 转换为 PX4(NED) 加速度指令
     // NED.acc.x (North) = ENU.acc.y (ay_enu)
     // NED.acc.y (East)  = ENU.acc.x (ax_enu)
     // NED.acc.z (Down)  = -ENU.acc.z (-az_enu)
     msg.acceleration = {
         static_cast<float>(ay_enu), 
         static_cast<float>(ax_enu), 
         static_cast<float>(-az_enu)
     }; 
     
     msg.yaw = static_cast<float>(yaw_ref); 
     
     trajectory_pub_->publish(msg);
   }
 
 
   // Member variables
   std::unique_ptr<ladrc_controller::LADRCController> ladrc_x_;
   std::unique_ptr<ladrc_controller::LADRCController> ladrc_y_;
   std::unique_ptr<ladrc_controller::LADRCController> ladrc_z_;
 
   rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr reference_sub_;
   rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_; 
   
   rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
   rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
   rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
 
   rclcpp::TimerBase::SharedPtr control_timer_;
 
   geometry_msgs::msg::PoseStamped reference_pose_;
   px4_msgs::msg::VehicleOdometry current_odom_; 
 
   bool has_reference_ = false;
   bool has_odom_ = false;
   double dt_;
   double max_acc_; // [修正]
   int log_counter_ = 0;
 };
 
 int main(int argc, char** argv)
 {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<LADRCPositionControllerNode>());
   rclcpp::shutdown();
   return 0;
 }