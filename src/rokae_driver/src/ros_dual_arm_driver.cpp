#include <iostream>
#include <memory>
#include <vector>

#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rokae_driver/node_factories.hpp"
#include "rokae_driver/shared_arm_hardware.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;

  try {
    auto coordinator =
        std::make_shared<rclcpp::Node>("rokae_dual_arm_driver");
    const bool start_hand_service = coordinator->declare_parameter<bool>(
        "start_hand_service", true);
    const bool start_go_home_service = coordinator->declare_parameter<bool>(
        "start_go_home_service", true);
    const bool start_initializer_service =
        coordinator->declare_parameter<bool>(
            "start_initializer_service", true);
    const bool start_move_server = coordinator->declare_parameter<bool>(
        "start_move_server", true);
    const bool start_movel_service = coordinator->declare_parameter<bool>(
        "start_movel_service", true);
    const bool start_state_publisher = coordinator->declare_parameter<bool>(
        "start_state_publisher", true);
    const bool start_servoj = coordinator->declare_parameter<bool>(
        "start_servoj", true);

    std::vector<std::shared_ptr<rclcpp::Node>> nodes;
    nodes.push_back(coordinator);

    // Construct state publishers first. Their network parameters become the
    // authoritative pair checked by every other interface node.
    if (start_state_publisher) {
      auto state_nodes = rokae_driver::makeStatePublishers();
      nodes.insert(nodes.end(), state_nodes.begin(), state_nodes.end());
    }
    if (start_initializer_service) {
      nodes.push_back(rokae_driver::makeRobotInitializerService());
    }
    if (start_move_server) {
      nodes.push_back(rokae_driver::makeMoveAbsJActionServer());
    }
    if (start_movel_service) {
      nodes.push_back(rokae_driver::makeMoveLService());
    }
    if (start_hand_service) {
      nodes.push_back(rokae_driver::makeHandService());
    }
    if (start_go_home_service) {
      nodes.push_back(rokae_driver::makeGoHomeService());
    }
    if (start_servoj) {
      nodes.push_back(rokae_driver::makeServoJSubscriber());
    }

    rclcpp::executors::MultiThreadedExecutor executor;
    for (const auto &node : nodes) executor.add_node(node);

    RCLCPP_INFO(
        coordinator->get_logger(),
        "Unified dual-arm driver started with one shared ArRobot instance "
        "per arm");
    executor.spin();

    for (const auto &node : nodes) executor.remove_node(node);
    nodes.clear();
  } catch (const std::exception &error) {
    std::cerr << "Unified dual-arm driver exception: " << error.what()
              << std::endl;
    exit_code = 1;
  }

  rokae_driver::SharedArmRegistry::instance().clear();
  if (rclcpp::ok()) rclcpp::shutdown();
  return exit_code;
}
