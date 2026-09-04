#ifndef ROKAE_DRIVER__NODE_FACTORIES_HPP_
#define ROKAE_DRIVER__NODE_FACTORIES_HPP_

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace rokae_driver {

std::shared_ptr<rclcpp::Node> makeHandService();
std::shared_ptr<rclcpp::Node> makeGoHomeService();
std::shared_ptr<rclcpp::Node> makeMoveAbsJActionServer();
std::shared_ptr<rclcpp::Node> makeMoveLService();
std::vector<std::shared_ptr<rclcpp::Node>> makeStatePublishers();
std::shared_ptr<rclcpp::Node> makeRobotInitializerService();
std::shared_ptr<rclcpp::Node> makeServoJSubscriber();

}  // namespace rokae_driver

#endif  // ROKAE_DRIVER__NODE_FACTORIES_HPP_
