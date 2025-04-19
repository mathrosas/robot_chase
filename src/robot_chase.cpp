#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using std::placeholders::_1;

class RobotChaseNode : public rclcpp::Node {
public:
  RobotChaseNode() : Node("robot_chase") {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("rick/cmd_vel", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&RobotChaseNode::chase_callback, this));

    kp_yaw_ = 2.0;
    kp_distance_ = 0.8;
  }

private:
  void chase_callback() {
    geometry_msgs::msg::TransformStamped transform;

    try {
      transform = tf_buffer_->lookupTransform(
          "rick/base_link", "morty/base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
      return;
    }

    double dx = transform.transform.translation.x;
    double dy = transform.transform.translation.y;

    double error_distance = std::sqrt(dx * dx + dy * dy);
    double error_yaw = std::atan2(dy, dx);

    double linear_velocity = kp_distance_ * error_distance;
    double angular_velocity = kp_yaw_ * error_yaw;

    // Stop if very close
    if (error_distance < 0.05) {
      linear_velocity = 0.0;
      angular_velocity = 0.0;
    }

    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = linear_velocity;
    twist_msg.angular.z = angular_velocity;

    cmd_vel_pub_->publish(twist_msg);
  }

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double kp_yaw_;
  double kp_distance_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotChaseNode>());
  rclcpp::shutdown();
  return 0;
}
