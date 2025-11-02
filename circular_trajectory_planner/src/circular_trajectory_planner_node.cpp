#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>

class CircularTrajectoryPlannerNode : public rclcpp::Node
{
public:
  CircularTrajectoryPlannerNode()
    : Node("circular_trajectory_planner"), time_(0.0)
  {
    // Declare parameters
    this->declare_parameter("center_x", 0.0);
    this->declare_parameter("center_y", 0.0);
    this->declare_parameter("center_z", 2.0);
    this->declare_parameter("radius", 5.0);
    this->declare_parameter("angular_velocity", 0.5);  // rad/s
    this->declare_parameter("publish_rate", 50.0);      // Hz
    this->declare_parameter("visualize", true);

    // Get parameters
    center_x_ = this->get_parameter("center_x").as_double();
    center_y_ = this->get_parameter("center_y").as_double();
    center_z_ = this->get_parameter("center_z").as_double();
    radius_ = this->get_parameter("radius").as_double();
    angular_vel_ = this->get_parameter("angular_velocity").as_double();
    double publish_rate = this->get_parameter("publish_rate").as_double();
    visualize_ = this->get_parameter("visualize").as_bool();

    dt_ = 1.0 / publish_rate;

    // Publishers
    reference_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/reference_pose", 10);
    
    if (visualize_) {
      path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/planned_path", 10);
      
      marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/trajectory_marker", 10);
    }

    // Timer
    auto timer_period = std::chrono::duration<double>(dt_);
    timer_ = this->create_wall_timer(
      timer_period,
      std::bind(&CircularTrajectoryPlannerNode::publishTrajectory, this));

    // Publish initial path visualization
    if (visualize_) {
      publishPathVisualization();
      publishCircleMarker();
    }

    RCLCPP_INFO(this->get_logger(), 
      "Circular Trajectory Planner initialized: center=[%.2f, %.2f, %.2f], radius=%.2f, omega=%.2f rad/s",
      center_x_, center_y_, center_z_, radius_, angular_vel_);
  }

private:
  void publishTrajectory()
  {
    // Calculate current angle
    double theta = angular_vel_ * time_;
    time_ += dt_;

    // Calculate circular trajectory position
    geometry_msgs::msg::PoseStamped reference_pose;
    reference_pose.header.stamp = this->get_clock()->now();
    reference_pose.header.frame_id = "map";

    reference_pose.pose.position.x = center_x_ + radius_ * std::cos(theta);
    reference_pose.pose.position.y = center_y_ + radius_ * std::sin(theta);
    reference_pose.pose.position.z = center_z_;

    // Calculate yaw to face forward along the circle
    double yaw = theta + M_PI / 2.0;  // Tangent to the circle
    
    // Convert yaw to quaternion (simplified for yaw-only rotation)
    reference_pose.pose.orientation.x = 0.0;
    reference_pose.pose.orientation.y = 0.0;
    reference_pose.pose.orientation.z = std::sin(yaw / 2.0);
    reference_pose.pose.orientation.w = std::cos(yaw / 2.0);

    reference_pub_->publish(reference_pose);
  }

  void publishPathVisualization()
  {
    nav_msgs::msg::Path path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "map";

    // Generate full circle path for visualization
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

    // Circle properties
    marker.scale.x = 0.1;  // Line width
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // Generate circle points
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

    // Timer to republish marker periodically
    auto marker_timer = this->create_wall_timer(
      std::chrono::seconds(5),
      [this, marker]() mutable {
        marker.header.stamp = this->get_clock()->now();
        this->marker_pub_->publish(marker);
      });
  }

  // Member variables
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr reference_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

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