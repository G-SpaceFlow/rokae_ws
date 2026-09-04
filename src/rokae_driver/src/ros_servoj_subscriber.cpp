#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rokae_interfaces/msg/dual_arm_servo_j.hpp"
#include "rokae_interfaces/msg/servo_j.hpp"

#include "rokae/data_types.h"
#include "rokae/motion_control_rt.h"
#include "rokae/robot.h"
#include "rokae_driver/node_factories.hpp"
#include "rokae_driver/shared_arm_hardware.hpp"

namespace {

constexpr std::size_t kJointCount = 7;
using JointArray = std::array<double, kJointCount>;
using RtController = rokae::RtMotionControlCobot<kJointCount>;
using Clock = std::chrono::steady_clock;

struct Options {
  double periodS{0.02};
  double lookaheadS{0.02};
  double gain{0.0};
  double commandTimeoutS{0.10};
  double maxCommandStepRad{0.02};
  double softLimitMarginRad{0.08};
};

struct ArmContext {
  std::string side;
  std::shared_ptr<rokae_driver::SharedArmHardware> hardware;
};

struct SingleFrame {
  bool enable{false};
  JointArray positions{};
  Clock::time_point received{};
  std::uint64_t sequence{0};
};

struct DualFrame {
  bool enable{false};
  JointArray left{};
  JointArray right{};
  Clock::time_point received{};
  std::uint64_t sequence{0};
};

template <typename Frame>
struct CommandBuffer {
  std::mutex mutex;
  std::condition_variable condition;
  Frame latest;
  bool hasMessage{false};
};

struct ArmSession {
  std::shared_ptr<RtController> controller;
  std::array<double[2], kJointCount> softLimits{};
  JointArray lastCommand{};
  bool servoConfigured{false};
  bool moveStarted{false};
  bool realtimeMode{false};
  bool stateReceiving{false};
};

bool finite(const JointArray &values) {
  return std::all_of(
      values.begin(), values.end(),
      [](double value) { return std::isfinite(value); });
}

std::string sdkError(const std::string &operation,
                     const std::error_code &error) {
  return operation + " failed: code=" + std::to_string(error.value()) +
         ", message=" + error.message();
}

class RokaeServoJSubscriber : public rclcpp::Node {
 public:
  RokaeServoJSubscriber() : Node("rokae_servoj") {
    const auto leftRobotIp = declare_parameter<std::string>(
        "left_arm.robot_ip", "192.168.4.160");
    const auto leftLocalIp = declare_parameter<std::string>(
        "left_arm.local_ip", "192.168.4.10");
    const auto rightRobotIp = declare_parameter<std::string>(
        "right_arm.robot_ip", "192.168.2.160");
    const auto rightLocalIp = declare_parameter<std::string>(
        "right_arm.local_ip", "192.168.2.10");

    options_.periodS = declare_parameter<double>("period_s", 0.02);
    options_.lookaheadS =
        declare_parameter<double>("lookahead_s", 0.02);
    options_.gain = declare_parameter<double>("gain", 0.0);
    options_.commandTimeoutS =
        declare_parameter<double>("command_timeout_s", 0.10);
    options_.maxCommandStepRad =
        declare_parameter<double>("max_command_step_rad", 0.02);
    options_.softLimitMarginRad =
        declare_parameter<double>("soft_limit_margin_rad", 0.08);
    validateOptions();

    left_ = {"left", rokae_driver::sharedArm(
                         "left", leftRobotIp, leftLocalIp)};
    right_ = {"right", rokae_driver::sharedArm(
                           "right", rightRobotIp, rightLocalIp)};

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.best_effort();
    qos.durability_volatile();

    leftSubscription_ = create_subscription<
        rokae_interfaces::msg::ServoJ>(
        "/left_arm/servoj", qos,
        [this](const rokae_interfaces::msg::ServoJ::SharedPtr message) {
          updateSingle(leftBuffer_, message->enable, message->positions);
        });
    rightSubscription_ = create_subscription<
        rokae_interfaces::msg::ServoJ>(
        "/right_arm/servoj", qos,
        [this](const rokae_interfaces::msg::ServoJ::SharedPtr message) {
          updateSingle(rightBuffer_, message->enable, message->positions);
        });
    dualSubscription_ = create_subscription<
        rokae_interfaces::msg::DualArmServoJ>(
        "/dual_arm/servoj", qos,
        [this](
            const rokae_interfaces::msg::DualArmServoJ::SharedPtr message) {
          std::lock_guard<std::mutex> lock(dualBuffer_.mutex);
          dualBuffer_.latest.enable = message->enable;
          dualBuffer_.latest.left = message->left_positions;
          dualBuffer_.latest.right = message->right_positions;
          dualBuffer_.latest.received = Clock::now();
          ++dualBuffer_.latest.sequence;
          dualBuffer_.hasMessage = true;
          dualBuffer_.condition.notify_one();
        });

    leftThread_ = std::thread(
        [this] { runSingle(left_, leftBuffer_, "/left_arm/servoj"); });
    rightThread_ = std::thread(
        [this] { runSingle(right_, rightBuffer_, "/right_arm/servoj"); });
    dualThread_ = std::thread([this] { runDual(); });

    RCLCPP_WARN(
        get_logger(),
        "ServoJ ready on /left_arm/servoj, /right_arm/servoj and "
        "/dual_arm/servoj (period=%.3f s, watchdog=%.3f s). The driver "
        "does not power on either robot.",
        options_.periodS, options_.commandTimeoutS);
  }

  ~RokaeServoJSubscriber() override {
    running_.store(false);
    leftBuffer_.condition.notify_all();
    rightBuffer_.condition.notify_all();
    dualBuffer_.condition.notify_all();
    if (leftThread_.joinable()) leftThread_.join();
    if (rightThread_.joinable()) rightThread_.join();
    if (dualThread_.joinable()) dualThread_.join();
  }

 private:
  void validateOptions() const {
    if (!std::isfinite(options_.periodS) || options_.periodS < 0.001 ||
        options_.periodS > 0.1) {
      throw std::invalid_argument("period_s must be in [0.001, 0.1]");
    }
    if (!std::isfinite(options_.lookaheadS) ||
        options_.lookaheadS < options_.periodS ||
        options_.lookaheadS > 1.0) {
      throw std::invalid_argument(
          "lookahead_s must be in [period_s, 1.0]");
    }
    if (!std::isfinite(options_.gain) || options_.gain < 0.0 ||
        options_.gain > 1000.0) {
      throw std::invalid_argument("gain must be in [0, 1000]");
    }
    if (!std::isfinite(options_.commandTimeoutS) ||
        options_.commandTimeoutS < 2.0 * options_.periodS ||
        options_.commandTimeoutS > 2.0) {
      throw std::invalid_argument(
          "command_timeout_s must be in [2 * period_s, 2.0]");
    }
    if (!std::isfinite(options_.maxCommandStepRad) ||
        options_.maxCommandStepRad <= 0.0 ||
        options_.maxCommandStepRad > 0.3) {
      throw std::invalid_argument(
          "max_command_step_rad must be in (0, 0.3]");
    }
    if (!std::isfinite(options_.softLimitMarginRad) ||
        options_.softLimitMarginRad < 0.0 ||
        options_.softLimitMarginRad > 0.3) {
      throw std::invalid_argument(
          "soft_limit_margin_rad must be in [0, 0.3]");
    }
  }

  void updateSingle(CommandBuffer<SingleFrame> &buffer, bool enable,
                    const JointArray &positions) {
    std::lock_guard<std::mutex> lock(buffer.mutex);
    buffer.latest.enable = enable;
    buffer.latest.positions = positions;
    buffer.latest.received = Clock::now();
    ++buffer.latest.sequence;
    buffer.hasMessage = true;
    buffer.condition.notify_one();
  }

  bool validateTarget(const ArmContext &arm, const ArmSession &session,
                      const JointArray &target, bool first,
                      std::string &reason) const {
    if (!finite(target)) {
      reason = "joint target contains NaN or infinity";
      return false;
    }
    for (std::size_t index = 0; index < kJointCount; ++index) {
      const double lower = session.softLimits[index][0] +
                           options_.softLimitMarginRad;
      const double upper = session.softLimits[index][1] -
                           options_.softLimitMarginRad;
      if (lower >= upper || target[index] < lower ||
          target[index] > upper) {
        reason = arm.side + " J" + std::to_string(index + 1) +
                 " violates the controller soft-limit margin";
        return false;
      }
      const double step =
          std::abs(target[index] - session.lastCommand[index]);
      if (step > options_.maxCommandStepRad) {
        reason = arm.side + " J" + std::to_string(index + 1) +
                 (first ? " first target" : " target step") +
                 " exceeds max_command_step_rad";
        return false;
      }
    }
    return true;
  }

  bool prepareArm(const ArmContext &arm, ArmSession &session,
                  std::string &reason) {
    std::error_code error;
    const auto power = arm.hardware->withRobot(
        [&error](rokae::ArRobot &robot) { return robot.powerState(error); });
    if (error) {
      reason = sdkError("powerState", error);
      return false;
    }
    if (power != rokae::PowerState::on) {
      reason = arm.side +
               " robot is not powered on; call /initialize_robots first";
      return false;
    }

    error.clear();
    const auto state = arm.hardware->withRobot(
        [&error](rokae::ArRobot &robot) {
          return robot.operationState(error);
        });
    if (error) {
      reason = sdkError("operationState", error);
      return false;
    }
    if (state != rokae::OperationState::idle) {
      reason = arm.side + " robot must be idle before starting ServoJ";
      return false;
    }

    error.clear();
    const bool limitsEnabled = arm.hardware->withRobot(
        [&session, &error](rokae::ArRobot &robot) {
          return robot.getSoftLimit(session.softLimits, error);
        });
    if (error) {
      reason = sdkError("getSoftLimit", error);
      return false;
    }
    if (!limitsEnabled) {
      reason = arm.side + " controller soft limits are disabled";
      return false;
    }

    error.clear();
    session.lastCommand = arm.hardware->withRobot(
        [&error](rokae::ArRobot &robot) { return robot.jointPos(error); });
    if (error) {
      reason = sdkError("jointPos", error);
      return false;
    }
    if (!finite(session.lastCommand)) {
      reason = arm.side + " joint feedback contains NaN or infinity";
      return false;
    }
    return true;
  }

  bool startArm(const ArmContext &arm, ArmSession &session,
                std::string &reason) {
    std::error_code error;
    try {
      arm.hardware->withRobot([&](rokae::ArRobot &robot) {
        robot.setOperateMode(rokae::OperateMode::automatic, error);
        if (error) return;
        robot.setMotionControlMode(
            rokae::MotionControlMode::RtCommand, error);
        if (error) return;
        session.realtimeMode = true;
        session.controller = robot.getRtMotionController().lock();
        robot.startReceiveRobotState(
            std::chrono::milliseconds(1),
            {rokae::RtSupportedFields::jointPos_m});
        session.stateReceiving = true;
      });
      if (error) {
        reason = sdkError("enter realtime mode", error);
        return false;
      }
      if (!session.controller) {
        reason = arm.side + " getRtMotionController returned an empty handle";
        return false;
      }

      arm.hardware->withSdkLock([&] {
        session.controller->setServoJoint(
            options_.periodS, options_.lookaheadS, options_.gain, error);
      });
      if (error) {
        reason = sdkError("setServoJoint", error);
        return false;
      }
      session.servoConfigured = true;
      arm.hardware->withSdkLock([&] {
        session.controller->startMove(
            rokae::RtControllerMode::jointPosition);
      });
      session.moveStarted = true;
      return true;
    } catch (const std::exception &exception) {
      reason = arm.side + " ServoJ startup exception: " + exception.what();
      return false;
    }
  }

  bool sendArm(const ArmContext &arm, ArmSession &session,
               const JointArray &target, std::string &reason) {
    try {
      arm.hardware->withRobot([](rokae::ArRobot &robot) {
        robot.updateRobotState(Clock::duration::zero());
      });
      rokae::JointPosition command(
          std::vector<double>(target.begin(), target.end()));
      arm.hardware->withSdkLock(
          [&] { session.controller->sendCommand(command); });
      session.lastCommand = target;
      return true;
    } catch (const std::exception &exception) {
      reason = arm.side + " sendCommand exception: " + exception.what();
      return false;
    }
  }

  void stopArm(const ArmContext &arm, ArmSession &session) noexcept {
    if (session.controller) {
      if (session.moveStarted) {
        try {
          rokae::JointPosition command(std::vector<double>(
              session.lastCommand.begin(), session.lastCommand.end()));
          command.setFinished();
          arm.hardware->withSdkLock(
              [&] { session.controller->sendCommand(command); });
        } catch (const std::exception &exception) {
          RCLCPP_ERROR(
              get_logger(), "%s ServoJ finish command failed: %s",
              arm.side.c_str(), exception.what());
          try {
            arm.hardware->withSdkLock(
                [&] { session.controller->stopMove(); });
          } catch (const std::exception &stopException) {
            RCLCPP_ERROR(
                get_logger(), "%s ServoJ stopMove failed: %s",
                arm.side.c_str(), stopException.what());
          }
        }
      }
      if (session.servoConfigured) {
        arm.hardware->withSdkLock(
            [&] { session.controller->stopServoJoint(); });
      }
    }

    if (session.realtimeMode) {
      std::error_code error;
      arm.hardware->withRobot([&error, &session](rokae::ArRobot &robot) {
        if (session.stateReceiving) robot.stopReceiveRobotState();
        robot.setMotionControlMode(
            rokae::MotionControlMode::NrtCommand, error);
      });
      if (error) {
        RCLCPP_ERROR(
            get_logger(), "%s", sdkError(
                arm.side + " restore NrtCommand", error).c_str());
      }
    }
    session = ArmSession{};
  }

  template <typename Frame>
  bool waitForFirst(CommandBuffer<Frame> &buffer, Frame &frame,
                    std::uint64_t &consumed) {
    std::unique_lock<std::mutex> lock(buffer.mutex);
    buffer.condition.wait(lock, [&] {
      return !running_.load() ||
             (buffer.hasMessage &&
              buffer.latest.sequence != consumed);
    });
    if (!running_.load()) return false;
    frame = buffer.latest;
    consumed = frame.sequence;
    return true;
  }

  template <typename Frame>
  bool updateUntil(CommandBuffer<Frame> &buffer,
                   Clock::time_point deadline, Frame &frame,
                   std::uint64_t &consumed) {
    std::unique_lock<std::mutex> lock(buffer.mutex);
    buffer.condition.wait_until(lock, deadline, [&] {
      return !running_.load() ||
             buffer.latest.sequence != consumed;
    });
    if (!running_.load()) return false;
    if (buffer.hasMessage && buffer.latest.sequence != consumed) {
      frame = buffer.latest;
      consumed = frame.sequence;
    }
    return true;
  }

  void runSingle(ArmContext &arm, CommandBuffer<SingleFrame> &buffer,
                 const char *topic) {
    std::uint64_t consumed = 0;
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(options_.periodS));
    const auto watchdog = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(options_.commandTimeoutS));

    while (running_.load()) {
      SingleFrame frame;
      if (!waitForFirst(buffer, frame, consumed)) break;
      if (!frame.enable) continue;

      std::unique_lock<std::mutex> commandLock(
          arm.hardware->commandMutex(), std::try_to_lock);
      if (!commandLock.owns_lock()) {
        RCLCPP_WARN(
            get_logger(), "%s rejected: %s arm is occupied",
            topic, arm.side.c_str());
        continue;
      }

      ArmSession session;
      std::string reason;
      if (!prepareArm(arm, session, reason) ||
          !validateTarget(
              arm, session, frame.positions, true, reason) ||
          !startArm(arm, session, reason)) {
        RCLCPP_ERROR(get_logger(), "%s rejected: %s", topic,
                     reason.c_str());
        if (session.realtimeMode || session.controller) {
          stopArm(arm, session);
        }
        continue;
      }

      RCLCPP_INFO(get_logger(), "%s realtime session started", topic);
      bool sessionRunning = true;
      auto nextSend = Clock::now();
      while (running_.load() && sessionRunning) {
        if (!frame.enable) {
          RCLCPP_INFO(get_logger(), "%s received stop command", topic);
          break;
        }
        if (Clock::now() - frame.received > watchdog) {
          RCLCPP_ERROR(
              get_logger(), "%s command timeout; stopping ServoJ", topic);
          break;
        }
        if (!validateTarget(
                arm, session, frame.positions, false, reason) ||
            !sendArm(arm, session, frame.positions, reason)) {
          RCLCPP_ERROR(get_logger(), "%s stopped: %s", topic,
                       reason.c_str());
          break;
        }

        nextSend += period;
        sessionRunning = updateUntil(
            buffer, nextSend, frame, consumed);
      }
      stopArm(arm, session);
      RCLCPP_INFO(get_logger(), "%s realtime session stopped", topic);
    }
  }

  void runDual() {
    std::uint64_t consumed = 0;
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(options_.periodS));
    const auto watchdog = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(options_.commandTimeoutS));

    while (running_.load()) {
      DualFrame frame;
      if (!waitForFirst(dualBuffer_, frame, consumed)) break;
      if (!frame.enable) continue;

      std::unique_lock<std::mutex> leftLock(
          left_.hardware->commandMutex(), std::defer_lock);
      std::unique_lock<std::mutex> rightLock(
          right_.hardware->commandMutex(), std::defer_lock);
      if (std::try_lock(leftLock, rightLock) != -1) {
        RCLCPP_WARN(
            get_logger(),
            "/dual_arm/servoj rejected: at least one arm is occupied");
        continue;
      }

      ArmSession leftSession;
      ArmSession rightSession;
      std::string reason;
      bool leftStarted = false;
      bool rightStarted = false;
      bool ready =
          prepareArm(left_, leftSession, reason) &&
          prepareArm(right_, rightSession, reason) &&
          validateTarget(
              left_, leftSession, frame.left, true, reason) &&
          validateTarget(
              right_, rightSession, frame.right, true, reason);
      if (ready) {
        leftStarted = startArm(left_, leftSession, reason);
        ready = leftStarted;
      }
      if (ready) {
        rightStarted = startArm(right_, rightSession, reason);
        ready = rightStarted;
      }
      if (!ready) {
        RCLCPP_ERROR(
            get_logger(), "/dual_arm/servoj rejected: %s",
            reason.c_str());
        if (rightSession.realtimeMode || rightSession.controller) {
          stopArm(right_, rightSession);
        }
        if (leftSession.realtimeMode || leftSession.controller) {
          stopArm(left_, leftSession);
        }
        continue;
      }

      RCLCPP_INFO(
          get_logger(), "/dual_arm/servoj realtime session started");
      bool sessionRunning = true;
      auto nextSend = Clock::now();
      while (running_.load() && sessionRunning) {
        if (!frame.enable) {
          RCLCPP_INFO(
              get_logger(), "/dual_arm/servoj received stop command");
          break;
        }
        if (Clock::now() - frame.received > watchdog) {
          RCLCPP_ERROR(
              get_logger(),
              "/dual_arm/servoj command timeout; stopping both arms");
          break;
        }
        if (!validateTarget(
                left_, leftSession, frame.left, false, reason) ||
            !validateTarget(
                right_, rightSession, frame.right, false, reason) ||
            !sendArm(left_, leftSession, frame.left, reason) ||
            !sendArm(right_, rightSession, frame.right, reason)) {
          RCLCPP_ERROR(
              get_logger(), "/dual_arm/servoj stopped: %s",
              reason.c_str());
          break;
        }

        nextSend += period;
        sessionRunning = updateUntil(
            dualBuffer_, nextSend, frame, consumed);
      }
      if (rightStarted) stopArm(right_, rightSession);
      if (leftStarted) stopArm(left_, leftSession);
      RCLCPP_INFO(
          get_logger(), "/dual_arm/servoj realtime session stopped");
    }
  }

  Options options_;
  ArmContext left_;
  ArmContext right_;
  std::atomic<bool> running_{true};

  CommandBuffer<SingleFrame> leftBuffer_;
  CommandBuffer<SingleFrame> rightBuffer_;
  CommandBuffer<DualFrame> dualBuffer_;
  std::thread leftThread_;
  std::thread rightThread_;
  std::thread dualThread_;

  rclcpp::Subscription<rokae_interfaces::msg::ServoJ>::SharedPtr
      leftSubscription_;
  rclcpp::Subscription<rokae_interfaces::msg::ServoJ>::SharedPtr
      rightSubscription_;
  rclcpp::Subscription<
      rokae_interfaces::msg::DualArmServoJ>::SharedPtr
      dualSubscription_;
};

}  // namespace

namespace rokae_driver {

std::shared_ptr<rclcpp::Node> makeServoJSubscriber() {
  return std::make_shared<RokaeServoJSubscriber>();
}

}  // namespace rokae_driver
