/**
 * @file ros_hand_service.cpp
 * @brief Control Linker Hand O6/L6 devices through the Rokae end CAN bus.
 *
 * Services:
 *   /left_arm/control_hand
 *   /right_arm/control_hand
 *
 * This is the ROS 2 service form of control_hand.cpp. The Linker Hand
 * protocol uses one command byte followed by zero or six value bytes:
 *   0x01: M1-M6 positions
 *   0x05: M1-M6 speeds
 *   0x20: read five-finger normal pressure
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rokae_interfaces/srv/control_hand.hpp"
#include "rokae/robot.h"

namespace {

using ControlHand = rokae_interfaces::srv::ControlHand;

struct HandContext {
  std::string side;
  int canId{0};
  int receiveTimeoutMs{200};
  int receiveAttempts{3};
  int replyDelayMs{1000};
  std::unique_ptr<rokae::ArRobot> robot;
  std::mutex mutex;
  rclcpp::CallbackGroup::SharedPtr callbackGroup;
  rclcpp::Service<ControlHand>::SharedPtr service;
};

std::string normalizedCommand(std::string command) {
  std::transform(
      command.begin(), command.end(), command.begin(),
      [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
      });
  return command;
}

std::string sdkError(
    const std::string &operation, const std::error_code &error) {
  return operation + " failed: code=" + std::to_string(error.value()) +
         ", message=" + error.message();
}

std::string hexadecimalId(std::uint32_t frameId) {
  std::ostringstream stream;
  stream << "0x" << std::hex << std::uppercase << frameId;
  return stream.str();
}

bool buildPayload(
    const ControlHand::Request &request,
    std::vector<std::uint8_t> &payload,
    std::string &reason) {
  const std::string command = normalizedCommand(request.command);
  if (command == "open") {
    payload = {0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    return true;
  }
  if (command == "half") {
    payload = {0x01, 0xa0, 0xa0, 0xa0, 0xa0, 0xa0, 0xa0};
    return true;
  }
  if (command == "close") {
    payload = {0x01, 0x45, 0x45, 0x45, 0x45, 0x45, 0x45};
    return true;
  }
  if (command == "position") {
    payload = {
        0x01, request.values[0], request.values[0], request.values[0],
        request.values[0], request.values[0], request.values[0]};
    return true;
  }
  if (command == "motors" || command == "joints") {
    payload = {
        0x01, request.values[0], request.values[1], request.values[2],
        request.values[3], request.values[4], request.values[5]};
    return true;
  }
  if (command == "speed") {
    payload = {
        0x05, request.values[0], request.values[0], request.values[0],
        request.values[0], request.values[0], request.values[0]};
    return true;
  }
  if (command == "pressure") {
    payload = {0x20};
    return true;
  }

  reason =
      "command must be open, half, close, position, motors, joints, "
      "speed or pressure";
  return false;
}

}  // namespace

class RokaeHandService : public rclcpp::Node {
 public:
  RokaeHandService() : Node("rokae_hand_service") {
    left_ = createHand(
        "left", "192.168.0.160", "192.168.0.10", 0x28,
        "/left_arm/control_hand");
    right_ = createHand(
        "right", "192.168.2.160", "192.168.2.10", 0x27,
        "/right_arm/control_hand");

    RCLCPP_INFO(
        get_logger(),
        "Linker Hand services are ready; protocol positions and speeds "
        "use integer bytes in [0, 255]");
  }

 private:
  std::unique_ptr<HandContext> createHand(
      const std::string &side, const std::string &defaultRobotIp,
      const std::string &defaultLocalIp, int defaultCanId,
      const std::string &serviceName) {
    const std::string prefix = side + "_hand";
    const auto robotIp = declare_parameter<std::string>(
        prefix + ".robot_ip", defaultRobotIp);
    const auto localIp = declare_parameter<std::string>(
        prefix + ".local_ip", defaultLocalIp);
    const int canId = declare_parameter<int>(
        prefix + ".can_id", defaultCanId);
    const int receiveTimeoutMs = declare_parameter<int>(
        prefix + ".receive_timeout_ms", 200);
    const int receiveAttempts = declare_parameter<int>(
        prefix + ".receive_attempts", 3);
    const int replyDelayMs = declare_parameter<int>(
        prefix + ".reply_delay_ms", 1000);

    if (canId < 0 || canId > 0x7ff) {
      throw std::invalid_argument(
          prefix + ".can_id must be a standard CAN ID in [0, 2047]");
    }
    if (receiveTimeoutMs < 1 || receiveTimeoutMs > 5000) {
      throw std::invalid_argument(
          prefix + ".receive_timeout_ms must be in [1, 5000]");
    }
    if (receiveAttempts < 1 || receiveAttempts > 20) {
      throw std::invalid_argument(
          prefix + ".receive_attempts must be in [1, 20]");
    }
    if (replyDelayMs < 0 || replyDelayMs > 5000) {
      throw std::invalid_argument(
          prefix + ".reply_delay_ms must be in [0, 5000]");
    }

    auto hand = std::make_unique<HandContext>();
    hand->side = side;
    hand->canId = canId;
    hand->receiveTimeoutMs = receiveTimeoutMs;
    hand->receiveAttempts = receiveAttempts;
    hand->replyDelayMs = replyDelayMs;

    RCLCPP_INFO(
        get_logger(),
        "Connecting %s hand CAN channel: robot=%s, local=%s, CAN ID=%s",
        side.c_str(), robotIp.c_str(), localIp.c_str(),
        hexadecimalId(static_cast<std::uint32_t>(canId)).c_str());
    hand->robot = std::make_unique<rokae::ArRobot>(robotIp, localIp);
    hand->callbackGroup = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    HandContext *const handPointer = hand.get();
    hand->service = create_service<ControlHand>(
        serviceName,
        [this, handPointer](
            const std::shared_ptr<ControlHand::Request> request,
            std::shared_ptr<ControlHand::Response> response) {
          execute(*handPointer, *request, *response);
        },
        rmw_qos_profile_services_default,
        hand->callbackGroup);
    RCLCPP_INFO(get_logger(), "%s is ready", serviceName.c_str());
    return hand;
  }

  void execute(
      HandContext &hand, const ControlHand::Request &request,
      ControlHand::Response &response) {
    std::unique_lock<std::mutex> lock(hand.mutex, std::try_to_lock);
    if (!lock.owns_lock()) {
      fail(response, hand.side + " hand is already executing a request");
      return;
    }

    std::vector<std::uint8_t> payload;
    std::string reason;
    if (!buildPayload(request, payload, reason)) {
      fail(response, reason);
      return;
    }

    try {
      rokae::CANFrame frame;
      frame.frame_id = hand.canId;
      frame.frame_format =
          static_cast<int>(rokae::CANFormat::STANDARD);
      frame.frame_type = static_cast<int>(rokae::CANType::CAN);
      frame.frame_valid_length = static_cast<int>(payload.size());
      frame.data = payload;

      std::error_code error;
      hand.robot->CANSendData(
          "uint8", std::vector<rokae::CANFrame>{frame}, error);
      if (error) {
        fail(response, sdkError("CANSendData", error));
        return;
      }

      std::this_thread::sleep_for(
          std::chrono::milliseconds(hand.replyDelayMs));

      rokae::CANFrame received;
      for (int attempt = 1; attempt <= hand.receiveAttempts; ++attempt) {
        error.clear();
        hand.robot->CANReceiveData(
            hand.receiveTimeoutMs, "uint8", received, error);
        if (!error) {
          const bool expectedId =
              received.frame_id == hand.canId ||
              received.frame_id == hand.canId + 8;
          const bool expectedCommand =
              !received.data.empty() &&
              received.data.front() == payload.front();
          if (!expectedId || !expectedCommand) {
            RCLCPP_WARN(
                get_logger(),
                "%s hand ignored unrelated CAN reply id=%s",
                hand.side.c_str(),
                hexadecimalId(received.frame_id).c_str());
            continue;
          }

          response.success = true;
          response.frame_id =
              static_cast<std::uint32_t>(received.frame_id);
          response.data = received.data;
          response.message =
              hand.side + " hand " +
              normalizedCommand(request.command) +
              " completed; reply_id=" +
              hexadecimalId(response.frame_id);
          return;
        }

        if (attempt != hand.receiveAttempts) {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
      }

      fail(
          response,
          hand.side + " hand reply timed out after " +
              std::to_string(hand.receiveAttempts) +
              " receive attempts; " +
              sdkError("CANReceiveData", error));
    } catch (const std::exception &exception) {
      fail(
          response,
          hand.side + " hand CAN exception: " + exception.what());
    }
  }

  void fail(
      ControlHand::Response &response, const std::string &message) {
    response.success = false;
    response.message = message;
    response.frame_id = 0;
    response.data.clear();
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
  }

  std::unique_ptr<HandContext> left_;
  std::unique_ptr<HandContext> right_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<RokaeHandService>();
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception &exception) {
    RCLCPP_FATAL(
        rclcpp::get_logger("rokae_hand_service"), "%s",
        exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
