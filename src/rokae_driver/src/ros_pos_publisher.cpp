#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

#include "rokae/robot.h"
#include "rokae/utility.h"

class RokaeStatePublisher : public rclcpp::Node {
 public:
  RokaeStatePublisher(
      const std::string &node_namespace,
      const std::string &default_arm_name,
      const std::string &default_robot_ip,
      const std::string &default_local_ip,
      const std::string &default_frame_id)
      : Node("rokae_state_publisher", node_namespace) {
    arm_name_ =
        declare_parameter<std::string>("arm_name", default_arm_name);
    robot_ip_ =
        declare_parameter<std::string>("robot_ip", default_robot_ip);
    local_ip_ =
        declare_parameter<std::string>("local_ip", default_local_ip);
    frame_id_ =
        declare_parameter<std::string>("frame_id", default_frame_id);
    const double rate_hz =
        declare_parameter<double>("rate_hz", 20.0);
    if (!std::isfinite(rate_hz) || rate_hz <= 0.0 ||
        rate_hz > 200.0) {
      throw std::invalid_argument(
          "rate_hz must be a finite number in (0, 200]");
    }

    for (int i = 1; i <= 7; ++i) {
      joint_names_.push_back(
          arm_name_ + "_joint_" + std::to_string(i));
    }

    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::QoS(10));

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "tcp_pose", rclcpp::QoS(10));

    RCLCPP_INFO(
        get_logger(),
        "Connecting %s arm: robot=%s, local=%s",
        arm_name_.c_str(), robot_ip_.c_str(), local_ip_.c_str());

    // 整个节点运行期间只连接一次。
    robot_ = std::make_unique<rokae::ArRobot>(
        robot_ip_, local_ip_);

    const auto period = std::chrono::duration_cast<
        std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz));

    timer_ = create_wall_timer(
        period,
        std::bind(&RokaeStatePublisher::publishState, this));

    RCLCPP_INFO(
        get_logger(),
        "Publishing %s/joint_states and %s/tcp_pose at %.3f Hz",
        get_namespace(), get_namespace(), rate_hz);
  }

 private:
  void publishState() {
    std::error_code ec;

    const auto joints = robot_->jointPos(ec);
    if (ec) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "%s jointPos failed: code=%d, message=%s",
          arm_name_.c_str(),
          ec.value(), ec.message().c_str());
      return;
    }

    ec.clear();
    const auto pose = robot_->posture(
        rokae::CoordinateType::endInRef, ec);

    if (ec) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "%s posture failed: code=%d, message=%s",
          arm_name_.c_str(),
          ec.value(), ec.message().c_str());
      return;
    }

    const auto stamp = now();

    sensor_msgs::msg::JointState joint_msg;
    joint_msg.header.stamp = stamp;
    joint_msg.name = joint_names_;
    joint_msg.position.assign(joints.begin(), joints.end());
    joint_pub_->publish(joint_msg);

    // 通过SDK生成旋转矩阵，避免直接猜测RPY旋转顺序。
    std::array<double, 16> transform{};
    rokae::Utils::postureToTransArray(pose, transform);

    tf2::Matrix3x3 rotation(
        transform[0], transform[1], transform[2],
        transform[4], transform[5], transform[6],
        transform[8], transform[9], transform[10]);

    tf2::Quaternion quaternion;
    rotation.getRotation(quaternion);
    quaternion.normalize();

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = frame_id_;

    pose_msg.pose.position.x = pose[0];
    pose_msg.pose.position.y = pose[1];
    pose_msg.pose.position.z = pose[2];

    pose_msg.pose.orientation.x = quaternion.x();
    pose_msg.pose.orientation.y = quaternion.y();
    pose_msg.pose.orientation.z = quaternion.z();
    pose_msg.pose.orientation.w = quaternion.w();

    pose_pub_->publish(pose_msg);
  }

  std::string arm_name_;
  std::string robot_ip_;
  std::string local_ip_;
  std::string frame_id_;

  std::vector<std::string> joint_names_;
  std::unique_ptr<rokae::ArRobot> robot_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  int exit_code = 0;
  try {
    auto left = std::make_shared<RokaeStatePublisher>(
        "/left_arm", "left",
        "192.168.0.160", "192.168.0.10",
        "left_external_ref");
    auto right = std::make_shared<RokaeStatePublisher>(
        "/right_arm", "right",
        "192.168.2.160", "192.168.2.10",
        "right_external_ref");

    // 两个SDK读取定时器可独立执行，避免一侧网络等待阻塞另一侧。
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(left);
    executor.add_node(right);
    executor.spin();
  } catch (const std::exception &e) {
    std::cerr << "Node exception: " << e.what() << std::endl;
    exit_code = 1;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
