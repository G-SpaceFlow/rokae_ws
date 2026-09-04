/**
 * @file ros_go_home_service.cpp
 * @brief Safe single- and dual-arm MoveAbsJ services for known home poses.
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "rokae/data_types.h"
#include "rokae/robot.h"
#include "rokae_driver/node_factories.hpp"
#include "rokae_driver/shared_arm_hardware.hpp"

namespace {

constexpr std::size_t kJointCount = 7;
constexpr auto kMonitorPeriod = std::chrono::milliseconds(50);
constexpr int kSpeedViolationSamples = 2;
using JointArray = std::array<double, kJointCount>;

struct Options {
  double speedMmS{50.0};
  double jointSpeedScale{0.05};
  double zoneMm{0.0};
  double timeoutS{60.0};
  double softLimitMarginRad{0.08};
  double maxGoalDeltaRad{2.3};
  double maxJointSpeedRadS{0.40};
  double goalToleranceRad{0.01};
};

struct ArmContext {
  std::string side;
  JointArray home{};
  std::shared_ptr<rokae_driver::SharedArmHardware> hardware;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service;
};

struct ArmRunState {
  std::array<double[2], kJointCount> softLimits{};
  std::string commandId;
  int speedViolationCount{0};
  bool prepared{false};
  bool started{false};
  bool completed{false};
};

std::string sdkError(const std::string &operation,
                     const std::error_code &error) {
  return operation + " failed: code=" + std::to_string(error.value()) +
         ", message=" + error.message();
}

bool finite(const JointArray &values) {
  return std::all_of(values.begin(), values.end(), [](double value) {
    return std::isfinite(value);
  });
}

class RokaeGoHomeService : public rclcpp::Node {
 public:
  RokaeGoHomeService() : Node("rokae_go_home") {
    options_.speedMmS = declare_parameter<double>("speed_mm_s", 50.0);
    options_.jointSpeedScale =
        declare_parameter<double>("joint_speed_scale", 0.05);
    options_.zoneMm = declare_parameter<double>("zone_mm", 0.0);
    options_.timeoutS = declare_parameter<double>("timeout_s", 60.0);
    options_.softLimitMarginRad =
        declare_parameter<double>("soft_limit_margin_rad", 0.08);
    options_.maxGoalDeltaRad =
        declare_parameter<double>("max_goal_delta_rad", 2.3);
    options_.maxJointSpeedRadS =
        declare_parameter<double>("max_joint_speed_rad_s", 0.40);
    options_.goalToleranceRad =
        declare_parameter<double>("goal_tolerance_rad", 0.01);
    validateOptions();

    left_ = createArm(
        "left", "192.168.4.160", "192.168.4.10",
        {1.712167996, 1.570796327, -1.570796327, 0.0, 0.0, 0.0,
         0.0});
    right_ = createArm(
        "right", "192.168.2.160", "192.168.2.10",
        {1.7121, -1.570796327, -1.570796327, 0.0, 0.0, 0.0, 0.0});

    callbackGroup_ = create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);
    left_->service = create_service<std_srvs::srv::Trigger>(
        "/left_arm/go_home",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          handleSingle(*left_, *response);
        },
        rmw_qos_profile_services_default, callbackGroup_);
    right_->service = create_service<std_srvs::srv::Trigger>(
        "/right_arm/go_home",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          handleSingle(*right_, *response);
        },
        rmw_qos_profile_services_default, callbackGroup_);
    dualService_ = create_service<std_srvs::srv::Trigger>(
        "/dual_arm/go_home",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          handleDual(*response);
        },
        rmw_qos_profile_services_default, callbackGroup_);

    RCLCPP_WARN(
        get_logger(),
        "Go-home services ready on /left_arm/go_home, "
        "/right_arm/go_home and /dual_arm/go_home. Requests move hardware; "
        "this node does not power on either robot.");
  }

 private:
  std::unique_ptr<ArmContext> createArm(
      const std::string &side, const std::string &defaultRobotIp,
      const std::string &defaultLocalIp, const JointArray &defaultHome) {
    const std::string prefix = side + "_arm";
    const auto robotIp = declare_parameter<std::string>(
        prefix + ".robot_ip", defaultRobotIp);
    const auto localIp = declare_parameter<std::string>(
        prefix + ".local_ip", defaultLocalIp);
    const auto homeValues = declare_parameter<std::vector<double>>(
        prefix + ".home_positions",
        std::vector<double>(defaultHome.begin(), defaultHome.end()));
    if (homeValues.size() != kJointCount) {
      throw std::invalid_argument(
          prefix + ".home_positions must contain exactly 7 values");
    }

    auto arm = std::make_unique<ArmContext>();
    arm->side = side;
    std::copy(homeValues.begin(), homeValues.end(), arm->home.begin());
    if (!finite(arm->home)) {
      throw std::invalid_argument(
          prefix + ".home_positions contains NaN or infinity");
    }
    arm->hardware = rokae_driver::sharedArm(
        side, robotIp, localIp);
    return arm;
  }

  void validateOptions() const {
    if (!std::isfinite(options_.speedMmS) || options_.speedMmS <= 0.0 ||
        options_.speedMmS > 5000.0) {
      throw std::invalid_argument("speed_mm_s must be in (0, 5000]");
    }
    if (!std::isfinite(options_.jointSpeedScale) ||
        options_.jointSpeedScale <= 0.0 ||
        options_.jointSpeedScale > 1.0) {
      throw std::invalid_argument("joint_speed_scale must be in (0, 1]");
    }
    if (!std::isfinite(options_.zoneMm) || options_.zoneMm < 0.0 ||
        options_.zoneMm > 100.0) {
      throw std::invalid_argument("zone_mm must be in [0, 100]");
    }
    if (!std::isfinite(options_.timeoutS) || options_.timeoutS < 1.0 ||
        options_.timeoutS > 300.0) {
      throw std::invalid_argument("timeout_s must be in [1, 300]");
    }
    if (!std::isfinite(options_.softLimitMarginRad) ||
        options_.softLimitMarginRad < 0.0 ||
        options_.softLimitMarginRad > 0.30) {
      throw std::invalid_argument(
          "soft_limit_margin_rad must be in [0, 0.30]");
    }
    if (!std::isfinite(options_.maxGoalDeltaRad) ||
        options_.maxGoalDeltaRad <= 0.0 ||
        options_.maxGoalDeltaRad > M_PI) {
      throw std::invalid_argument("max_goal_delta_rad must be in (0, pi]");
    }
    if (!std::isfinite(options_.maxJointSpeedRadS) ||
        options_.maxJointSpeedRadS <= 0.0 ||
        options_.maxJointSpeedRadS > 3.0) {
      throw std::invalid_argument(
          "max_joint_speed_rad_s must be in (0, 3]");
    }
    if (!std::isfinite(options_.goalToleranceRad) ||
        options_.goalToleranceRad <= 0.0 ||
        options_.goalToleranceRad > 0.20) {
      throw std::invalid_argument(
          "goal_tolerance_rad must be in (0, 0.20]");
    }
  }

  bool validateArm(ArmContext &arm, ArmRunState &run,
                   std::string &reason) {
    std::error_code error;
    const auto power = arm.hardware->withRobot(
        [&error](rokae::ArRobot &robot) { return robot.powerState(error); });
    if (error) {
      reason = arm.side + " " + sdkError("powerState", error);
      return false;
    }
    if (power != rokae::PowerState::on) {
      reason = arm.side +
               " robot is not powered on; call /initialize_robots first";
      return false;
    }

    error.clear();
    const auto operation = arm.hardware->withRobot(
        [&error](rokae::ArRobot &robot) {
          return robot.operationState(error);
        });
    if (error) {
      reason = arm.side + " " + sdkError("operationState", error);
      return false;
    }
    if (operation != rokae::OperationState::idle) {
      reason = arm.side + " robot must be idle before going home";
      return false;
    }

    error.clear();
    const bool limitsEnabled = arm.hardware->withRobot(
        [&run, &error](rokae::ArRobot &robot) {
          return robot.getSoftLimit(run.softLimits, error);
        });
    if (error) {
      reason = arm.side + " " + sdkError("getSoftLimit", error);
      return false;
    }
    if (!limitsEnabled) {
      reason = arm.side + " controller soft limits are disabled";
      return false;
    }

    error.clear();
    const JointArray current = arm.hardware->withRobot(
        [&error](rokae::ArRobot &robot) { return robot.jointPos(error); });
    if (error || !finite(current)) {
      reason = error ? arm.side + " " + sdkError("jointPos", error)
                     : arm.side + " joint feedback is not finite";
      return false;
    }

    for (std::size_t index = 0; index < kJointCount; ++index) {
      const double lower = run.softLimits[index][0] +
                           options_.softLimitMarginRad;
      const double upper = run.softLimits[index][1] -
                           options_.softLimitMarginRad;
      if (lower >= upper || arm.home[index] < lower ||
          arm.home[index] > upper) {
        reason = arm.side + " home J" + std::to_string(index + 1) +
                 " violates the controller soft-limit margin";
        return false;
      }
      const double delta = std::abs(arm.home[index] - current[index]);
      if (delta > options_.maxGoalDeltaRad) {
        reason = arm.side + " J" + std::to_string(index + 1) +
                 " home delta exceeds max_goal_delta_rad: " +
                 std::to_string(delta);
        return false;
      }
    }
    return true;
  }

  bool prepareArm(ArmContext &arm, ArmRunState &run,
                  std::string &reason) {
    std::error_code error;
    arm.hardware->withRobot([&](rokae::ArRobot &robot) {
      robot.setMotionControlMode(
          rokae::MotionControlMode::NrtCommand, error);
      if (error) return;
      robot.setOperateMode(rokae::OperateMode::automatic, error);
      if (error) return;
      robot.adjustSpeedOnline(1.0, error);
      if (error) return;
      robot.moveReset(error);
      if (error) return;

      rokae::MoveAbsJCommand command(
          rokae::JointPosition(std::vector<double>(
              arm.home.begin(), arm.home.end())),
          options_.speedMmS, options_.zoneMm);
      command.jointSpeed = options_.jointSpeedScale;
      robot.moveAppend({command}, run.commandId, error);
    });
    if (error) {
      reason = arm.side + " " + sdkError("prepare MoveAbsJ", error);
      return false;
    }
    run.prepared = true;
    return true;
  }

  bool startArm(ArmContext &arm, ArmRunState &run,
                std::string &reason) {
    std::error_code error;
    arm.hardware->withRobot(
        [&error](rokae::ArRobot &robot) { robot.moveStart(error); });
    if (error) {
      reason = arm.side + " " + sdkError("moveStart", error);
      return false;
    }
    run.started = true;
    RCLCPP_WARN(
        get_logger(), "%s go-home started: command_id=%s",
        arm.side.c_str(), run.commandId.c_str());
    return true;
  }

  void stopArm(ArmContext &arm) noexcept {
    std::error_code error;
    arm.hardware->withRobot(
        [&error](rokae::ArRobot &robot) { robot.moveReset(error); });
    if (error) {
      RCLCPP_ERROR(
          get_logger(), "%s", sdkError(
              arm.side + " moveReset", error).c_str());
    }
  }

  bool monitorArm(ArmContext &arm, ArmRunState &run,
                  std::string &reason) {
    if (run.completed) return true;

    std::error_code error;
    const JointArray joints = arm.hardware->withRobot(
        [&error](rokae::ArRobot &robot) { return robot.jointPos(error); });
    if (error || !finite(joints)) {
      reason = error ? arm.side + " " + sdkError("jointPos", error)
                     : arm.side + " joint feedback is not finite";
      return false;
    }

    error.clear();
    const JointArray velocities = arm.hardware->withRobot(
        [&error](rokae::ArRobot &robot) { return robot.jointVel(error); });
    if (error || !finite(velocities)) {
      reason = error ? arm.side + " " + sdkError("jointVel", error)
                     : arm.side + " joint velocity is not finite";
      return false;
    }

    bool speedExceeded = false;
    std::size_t fastestJoint = 0;
    double fastestSpeed = 0.0;
    for (std::size_t index = 0; index < kJointCount; ++index) {
      if (joints[index] < run.softLimits[index][0] ||
          joints[index] > run.softLimits[index][1]) {
        reason = arm.side + " J" + std::to_string(index + 1) +
                 " crossed the controller soft limit";
        return false;
      }
      const double speed = std::abs(velocities[index]);
      if (speed > fastestSpeed) {
        fastestSpeed = speed;
        fastestJoint = index;
      }
      if (speed > options_.maxJointSpeedRadS) speedExceeded = true;
    }
    run.speedViolationCount =
        speedExceeded ? run.speedViolationCount + 1 : 0;
    if (run.speedViolationCount >= kSpeedViolationSamples) {
      reason = arm.side + " J" + std::to_string(fastestJoint + 1) +
               " speed exceeded max_joint_speed_rad_s: " +
               std::to_string(fastestSpeed);
      return false;
    }

    error.clear();
    const auto operation = arm.hardware->withRobot(
        [&error](rokae::ArRobot &robot) {
          return robot.operationState(error);
        });
    if (error) {
      reason = arm.side + " " + sdkError("operationState", error);
      return false;
    }
    if (operation == rokae::OperationState::unknown) {
      reason = arm.side + " controller entered unknown operation state";
      return false;
    }
    if (operation != rokae::OperationState::idle) return true;

    double maximumError = 0.0;
    for (std::size_t index = 0; index < kJointCount; ++index) {
      maximumError = std::max(
          maximumError, std::abs(arm.home[index] - joints[index]));
    }
    if (maximumError > options_.goalToleranceRad) {
      reason = arm.side +
               " robot became idle outside goal tolerance; maximum error=" +
               std::to_string(maximumError);
      return false;
    }
    run.completed = true;
    return true;
  }

  bool execute(std::vector<ArmContext *> arms,
               std::string &message) {
    std::vector<ArmRunState> runs(arms.size());
    for (std::size_t index = 0; index < arms.size(); ++index) {
      if (!validateArm(*arms[index], runs[index], message)) return false;
    }
    for (std::size_t index = 0; index < arms.size(); ++index) {
      if (!prepareArm(*arms[index], runs[index], message)) {
        for (std::size_t prepared = 0; prepared <= index; ++prepared) {
          if (runs[prepared].prepared) stopArm(*arms[prepared]);
        }
        return false;
      }
    }
    for (std::size_t index = 0; index < arms.size(); ++index) {
      if (!startArm(*arms[index], runs[index], message)) {
        for (std::size_t stop = 0; stop < arms.size(); ++stop) {
          if (runs[stop].prepared) stopArm(*arms[stop]);
        }
        return false;
      }
    }

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration_cast<
                              std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(
                                  options_.timeoutS));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    while (rclcpp::ok()) {
      if (std::chrono::steady_clock::now() >= deadline) {
        message = "go-home timed out";
        for (auto *arm : arms) stopArm(*arm);
        return false;
      }

      bool complete = true;
      for (std::size_t index = 0; index < arms.size(); ++index) {
        if (!monitorArm(*arms[index], runs[index], message)) {
          for (auto *arm : arms) stopArm(*arm);
          return false;
        }
        complete = complete && runs[index].completed;
      }
      if (complete) {
        message = arms.size() == 2
                      ? "both arms reached their home positions"
                      : arms.front()->side +
                            " arm reached its home position";
        return true;
      }
      std::this_thread::sleep_for(kMonitorPeriod);
    }

    message = "ROS shutdown interrupted go-home";
    for (auto *arm : arms) stopArm(*arm);
    return false;
  }

  void handleSingle(ArmContext &arm,
                    std_srvs::srv::Trigger::Response &response) {
    std::unique_lock<std::mutex> commandLock(
        arm.hardware->commandMutex(), std::try_to_lock);
    if (!commandLock.owns_lock()) {
      response.success = false;
      response.message = arm.side + " arm is occupied";
      return;
    }
    try {
      response.success = execute({&arm}, response.message);
    } catch (const std::exception &exception) {
      stopArm(arm);
      response.success = false;
      response.message = std::string("go-home exception: ") +
                         exception.what();
    }
  }

  void handleDual(std_srvs::srv::Trigger::Response &response) {
    std::unique_lock<std::mutex> leftLock(
        left_->hardware->commandMutex(), std::defer_lock);
    std::unique_lock<std::mutex> rightLock(
        right_->hardware->commandMutex(), std::defer_lock);
    if (std::try_lock(leftLock, rightLock) != -1) {
      response.success = false;
      response.message = "at least one arm is occupied";
      return;
    }
    try {
      response.success = execute(
          {left_.get(), right_.get()}, response.message);
    } catch (const std::exception &exception) {
      stopArm(*left_);
      stopArm(*right_);
      response.success = false;
      response.message = std::string("dual go-home exception: ") +
                         exception.what();
    }
  }

  Options options_;
  std::unique_ptr<ArmContext> left_;
  std::unique_ptr<ArmContext> right_;
  rclcpp::CallbackGroup::SharedPtr callbackGroup_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr dualService_;
};

}  // namespace

namespace rokae_driver {

std::shared_ptr<rclcpp::Node> makeGoHomeService() {
  return std::make_shared<RokaeGoHomeService>();
}

}  // namespace rokae_driver
