/**
 * @file ros_moveabsj_action_server.cpp
 * @brief Two ROS 2 FollowJointTrajectory action servers backed by MoveAbsJ.
 *
 * Only one trajectory point is accepted. The point positions are converted to
 * one SDK MoveAbsJCommand. Speed and safety limits are ROS 2 parameters so
 * they can be changed at runtime without rebuilding this executable.
 *
 * Actions:
 *   /left_arm/move_absj
 *   /right_arm/move_absj
 *
 * This node deliberately does not power on either arm. Put each controller in
 * a known NRT/automatic/powered state before sending a goal.
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "rokae_driver/node_factories.hpp"
#include "rokae_driver/shared_arm_hardware.hpp"
#include "rokae/robot.h"

using FollowJointTrajectory =
    control_msgs::action::FollowJointTrajectory;
using GoalHandle =
    rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

namespace {

constexpr std::size_t kJointCount = 7;
constexpr auto kFeedbackPeriod = std::chrono::milliseconds(50);
constexpr int kSpeedViolationSamples = 2;

std::string sdkError(const std::string &operation,
                     const std::error_code &ec) {
  return operation + " failed: code=" + std::to_string(ec.value()) +
         ", message=" + ec.message();
}

bool finiteArray(const std::array<double, kJointCount> &values) {
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}

struct RuntimeOptions {
  double speedMmS{80.0};
  double jointSpeedScale{0.10};
  double zoneMm{0.0};
  double timeoutS{60.0};
  double softLimitMarginRad{0.08};
  double maxGoalDeltaRad{0.35};
  double maxJointSpeedRadS{0.60};
  double goalToleranceRad{0.01};
};

struct ArmContext {
  std::string side;
  std::string parameterPrefix;
  std::string actionName;
  std::array<std::string, kJointCount> jointNames;
  std::shared_ptr<rokae_driver::SharedArmHardware> hardware;
  std::atomic_bool active{false};
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr server;
};

class ActiveGoalGuard {
 public:
  explicit ActiveGoalGuard(std::atomic_bool &active) : active_(active) {}
  ~ActiveGoalGuard() { active_.store(false); }

 private:
  std::atomic_bool &active_;
};

}  // namespace

class RokaeMoveAbsJActionServer : public rclcpp::Node {
 public:
  RokaeMoveAbsJActionServer()
      : Node("rokae_moveabsj_action_server") {
    left_ = createArm(
        "left", "192.168.4.160", "192.168.4.10",
        "/left_arm/move_absj");
    right_ = createArm(
        "right", "192.168.2.160", "192.168.2.10",
        "/right_arm/move_absj");

    createActionServer(*left_);
    createActionServer(*right_);

    RCLCPP_WARN(
        get_logger(),
        "MoveAbsJ action servers are ready. This node will not power on the "
        "robots. First tests must use one arm, a small target change, low "
        "joint_speed_scale and an accessible E-stop.");
  }

  ~RokaeMoveAbsJActionServer() override {
    joinExecutionThreads();
  }

 private:
  std::unique_ptr<ArmContext> createArm(
      const std::string &side, const std::string &defaultRobotIp,
      const std::string &defaultLocalIp, const std::string &actionName) {
    const std::string prefix = side + "_arm";
    const std::string robotIp = declare_parameter<std::string>(
        prefix + ".robot_ip", defaultRobotIp);
    const std::string localIp = declare_parameter<std::string>(
        prefix + ".local_ip", defaultLocalIp);

    declare_parameter<double>(prefix + ".speed_mm_s", 80.0);
    declare_parameter<double>(prefix + ".joint_speed_scale", 0.10);
    declare_parameter<double>(prefix + ".zone_mm", 0.0);
    declare_parameter<double>(prefix + ".timeout_s", 60.0);
    declare_parameter<double>(prefix + ".soft_limit_margin_rad", 0.08);
    declare_parameter<double>(prefix + ".max_goal_delta_rad", 0.35);
    declare_parameter<double>(prefix + ".max_joint_speed_rad_s", 0.60);
    declare_parameter<double>(prefix + ".goal_tolerance_rad", 0.01);

    auto arm = std::make_unique<ArmContext>();
    arm->side = side;
    arm->parameterPrefix = prefix;
    arm->actionName = actionName;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      arm->jointNames[i] =
          side + "_joint_" + std::to_string(i + 1);
    }

    RCLCPP_INFO(
        get_logger(), "Using shared %s arm: robot=%s, local=%s",
        side.c_str(), robotIp.c_str(), localIp.c_str());
    arm->hardware = rokae_driver::sharedArm(side, robotIp, localIp);
    return arm;
  }

  void createActionServer(ArmContext &arm) {
    ArmContext *const armPtr = &arm;
    arm.server = rclcpp_action::create_server<FollowJointTrajectory>(
        this, arm.actionName,
        [this, armPtr](
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const FollowJointTrajectory::Goal> goal) {
          return handleGoal(*armPtr, uuid, std::move(goal));
        },
        [this, armPtr](const std::shared_ptr<GoalHandle> goalHandle) {
          return handleCancel(*armPtr, goalHandle);
        },
        [this, armPtr](const std::shared_ptr<GoalHandle> goalHandle) {
          handleAccepted(*armPtr, goalHandle);
        });

    RCLCPP_INFO(
        get_logger(), "%s action ready; expected joints: %s_joint_1..7",
        arm.actionName.c_str(), arm.side.c_str());
  }

  rclcpp_action::GoalResponse handleGoal(
      ArmContext &arm, const rclcpp_action::GoalUUID &,
      std::shared_ptr<const FollowJointTrajectory::Goal> goal) {
    std::array<double, kJointCount> target{};
    std::string reason;
    if (!extractTarget(arm, *goal, target, reason)) {
      RCLCPP_WARN(
          get_logger(), "Rejecting %s goal: %s",
          arm.side.c_str(), reason.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    bool expected = false;
    if (!arm.active.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(
          get_logger(), "Rejecting %s goal: another goal is active",
          arm.side.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
      ArmContext &arm, const std::shared_ptr<GoalHandle>) {
    RCLCPP_WARN(
        get_logger(), "Cancel requested for %s arm", arm.side.c_str());
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(
      ArmContext &arm, const std::shared_ptr<GoalHandle> goalHandle) {
    std::lock_guard<std::mutex> lock(executionThreadsMutex_);
    executionThreads_.emplace_back(
        &RokaeMoveAbsJActionServer::executeGoal, this,
        std::ref(arm), goalHandle);
  }

  bool extractTarget(
      const ArmContext &arm, const FollowJointTrajectory::Goal &goal,
      std::array<double, kJointCount> &target,
      std::string &reason) const {
    const auto &trajectory = goal.trajectory;
    if (trajectory.points.size() != 1) {
      reason = "exactly one trajectory point is required";
      return false;
    }
    if (trajectory.joint_names.size() != kJointCount) {
      reason = "exactly seven joint names are required";
      return false;
    }

    const auto &point = trajectory.points.front();
    if (point.positions.size() != kJointCount) {
      reason = "the trajectory point must contain seven positions";
      return false;
    }
    if (!point.velocities.empty() || !point.accelerations.empty() ||
        !point.effort.empty()) {
      reason =
          "velocities, accelerations and effort are unsupported; configure "
          "MoveAbsJ speed with ROS parameters";
      return false;
    }
    if (point.time_from_start.sec != 0 ||
        point.time_from_start.nanosec != 0) {
      reason =
          "time_from_start is unsupported by the MoveAbsJ bridge";
      return false;
    }

    for (std::size_t expected = 0; expected < kJointCount; ++expected) {
      const auto it = std::find(
          trajectory.joint_names.begin(),
          trajectory.joint_names.end(), arm.jointNames[expected]);
      if (it == trajectory.joint_names.end()) {
        reason = "missing joint name " + arm.jointNames[expected];
        return false;
      }
      const std::size_t source = static_cast<std::size_t>(
          std::distance(trajectory.joint_names.begin(), it));
      target[expected] = point.positions[source];
    }

    if (!finiteArray(target)) {
      reason = "joint positions must all be finite";
      return false;
    }
    return true;
  }

  RuntimeOptions readOptions(const ArmContext &arm) const {
    RuntimeOptions options;
    const std::string &prefix = arm.parameterPrefix;
    options.speedMmS =
        get_parameter(prefix + ".speed_mm_s").as_double();
    options.jointSpeedScale =
        get_parameter(prefix + ".joint_speed_scale").as_double();
    options.zoneMm =
        get_parameter(prefix + ".zone_mm").as_double();
    options.timeoutS =
        get_parameter(prefix + ".timeout_s").as_double();
    options.softLimitMarginRad =
        get_parameter(prefix + ".soft_limit_margin_rad").as_double();
    options.maxGoalDeltaRad =
        get_parameter(prefix + ".max_goal_delta_rad").as_double();
    options.maxJointSpeedRadS =
        get_parameter(prefix + ".max_joint_speed_rad_s").as_double();
    options.goalToleranceRad =
        get_parameter(prefix + ".goal_tolerance_rad").as_double();
    return options;
  }

  bool validateOptions(
      const RuntimeOptions &options, std::string &reason) const {
    const std::array<double, 8> values = {
        options.speedMmS,
        options.jointSpeedScale,
        options.zoneMm,
        options.timeoutS,
        options.softLimitMarginRad,
        options.maxGoalDeltaRad,
        options.maxJointSpeedRadS,
        options.goalToleranceRad,
    };
    if (!std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); })) {
      reason = "all runtime parameters must be finite";
      return false;
    }
    if (options.speedMmS <= 0.0 || options.speedMmS > 4000.0) {
      reason = "speed_mm_s must be in (0, 4000]";
      return false;
    }
    if (options.jointSpeedScale < 0.01 ||
        options.jointSpeedScale > 1.0) {
      reason = "joint_speed_scale must be in [0.01, 1.0]";
      return false;
    }
    if (options.zoneMm < 0.0 || options.zoneMm > 200.0) {
      reason = "zone_mm must be in [0, 200]";
      return false;
    }
    if (options.timeoutS < 1.0 || options.timeoutS > 300.0) {
      reason = "timeout_s must be in [1, 300]";
      return false;
    }
    if (options.softLimitMarginRad < 0.0 ||
        options.softLimitMarginRad > 0.30) {
      reason = "soft_limit_margin_rad must be in [0, 0.30]";
      return false;
    }
    if (options.maxGoalDeltaRad <= 0.0 ||
        options.maxGoalDeltaRad > 3.14159265358979323846) {
      reason = "max_goal_delta_rad must be in (0, pi]";
      return false;
    }
    if (options.maxJointSpeedRadS <= 0.0 ||
        options.maxJointSpeedRadS > 3.0) {
      reason = "max_joint_speed_rad_s must be in (0, 3.0]";
      return false;
    }
    if (options.goalToleranceRad <= 0.0 ||
        options.goalToleranceRad > 0.20) {
      reason = "goal_tolerance_rad must be in (0, 0.20]";
      return false;
    }
    return true;
  }

  bool validateRobotAndTarget(
      ArmContext &arm,
      const std::array<double, kJointCount> &target,
      const RuntimeOptions &options,
      std::array<double[2], kJointCount> &softLimits,
      std::array<double, kJointCount> &startJoints,
      std::string &reason) {
    std::error_code ec;
    const auto power = arm.hardware->withRobot(
        [&ec](rokae::ArRobot &robot) { return robot.powerState(ec); });
    if (ec) {
      reason = sdkError("powerState", ec);
      return false;
    }
    if (power != rokae::PowerState::on) {
      reason =
          "robot is not powered on; initialize it explicitly before moving";
      return false;
    }

    const auto state = arm.hardware->withRobot(
        [&ec](rokae::ArRobot &robot) {
          return robot.operationState(ec);
        });
    if (ec) {
      reason = sdkError("operationState", ec);
      return false;
    }
    if (state != rokae::OperationState::idle) {
      reason = "robot must be idle before accepting MoveAbsJ";
      return false;
    }

    const bool softLimitEnabled = arm.hardware->withRobot(
        [&softLimits, &ec](rokae::ArRobot &robot) {
          return robot.getSoftLimit(softLimits, ec);
        });
    if (ec) {
      reason = sdkError("getSoftLimit", ec);
      return false;
    }
    if (!softLimitEnabled) {
      reason = "controller soft limits are disabled";
      return false;
    }

    startJoints = arm.hardware->withRobot(
        [&ec](rokae::ArRobot &robot) { return robot.jointPos(ec); });
    if (ec) {
      reason = sdkError("jointPos", ec);
      return false;
    }
    if (!finiteArray(startJoints)) {
      reason = "current joint feedback contains a non-finite value";
      return false;
    }

    for (std::size_t i = 0; i < kJointCount; ++i) {
      const double lower =
          softLimits[i][0] + options.softLimitMarginRad;
      const double upper =
          softLimits[i][1] - options.softLimitMarginRad;
      if (lower >= upper) {
        reason = "invalid soft-limit interval on J" +
                 std::to_string(i + 1);
        return false;
      }
      if (target[i] < lower || target[i] > upper) {
        reason = "J" + std::to_string(i + 1) +
                 " target violates the controller soft limit margin";
        return false;
      }
      if (startJoints[i] < softLimits[i][0] ||
          startJoints[i] > softLimits[i][1]) {
        reason = "J" + std::to_string(i + 1) +
                 " is already outside the controller soft limits";
        return false;
      }
      const double requestedDelta =
          std::abs(target[i] - startJoints[i]);
      if (requestedDelta > options.maxGoalDeltaRad) {
        reason = "J" + std::to_string(i + 1) +
                 " requested change exceeds max_goal_delta_rad: current=" +
                 std::to_string(startJoints[i]) +
                 " rad, target=" + std::to_string(target[i]) +
                 " rad, delta=" + std::to_string(requestedDelta) +
                 " rad, limit=" +
                 std::to_string(options.maxGoalDeltaRad) + " rad";
        return false;
      }
    }
    return true;
  }

  void executeGoal(
      ArmContext &arm, const std::shared_ptr<GoalHandle> goalHandle) {
    ActiveGoalGuard activeGuard(arm.active);

    auto result =
        std::make_shared<FollowJointTrajectory::Result>();
    std::unique_lock<std::mutex> commandLock(
        arm.hardware->commandMutex(), std::try_to_lock);
    if (!commandLock.owns_lock()) {
      abortGoal(
          goalHandle, result,
          FollowJointTrajectory::Result::INVALID_GOAL,
          arm.side + " arm is occupied by another control command");
      return;
    }
    std::array<double, kJointCount> target{};
    std::string reason;

    try {
      if (!extractTarget(arm, *goalHandle->get_goal(), target, reason)) {
        abortGoal(goalHandle, result,
                  FollowJointTrajectory::Result::INVALID_GOAL, reason);
        return;
      }

      const RuntimeOptions options = readOptions(arm);
      if (!validateOptions(options, reason)) {
        abortGoal(goalHandle, result,
                  FollowJointTrajectory::Result::INVALID_GOAL, reason);
        return;
      }

      std::array<double[2], kJointCount> softLimits{};
      std::array<double, kJointCount> startJoints{};
      if (!validateRobotAndTarget(
              arm, target, options, softLimits, startJoints, reason)) {
        abortGoal(goalHandle, result,
                  FollowJointTrajectory::Result::INVALID_GOAL, reason);
        return;
      }

      std::error_code ec;
      arm.hardware->withRobot([&ec](rokae::ArRobot &robot) {
        robot.setMotionControlMode(
            rokae::MotionControlMode::NrtCommand, ec);
      });
      if (ec) {
        abortSdkGoal(goalHandle, result, arm, "setMotionControlMode", ec);
        return;
      }
      arm.hardware->withRobot([&ec](rokae::ArRobot &robot) {
        robot.setOperateMode(rokae::OperateMode::automatic, ec);
      });
      if (ec) {
        abortSdkGoal(goalHandle, result, arm, "setOperateMode", ec);
        return;
      }
      arm.hardware->withRobot([&ec](rokae::ArRobot &robot) {
        robot.adjustSpeedOnline(1.0, ec);
      });
      if (ec) {
        abortSdkGoal(goalHandle, result, arm, "adjustSpeedOnline", ec);
        return;
      }
      arm.hardware->withRobot(
          [&ec](rokae::ArRobot &robot) { robot.moveReset(ec); });
      if (ec) {
        abortSdkGoal(goalHandle, result, arm, "moveReset", ec);
        return;
      }

      rokae::MoveAbsJCommand command(
          rokae::JointPosition(std::vector<double>(
              target.begin(), target.end())),
          options.speedMmS, options.zoneMm);
      command.jointSpeed = options.jointSpeedScale;

      std::string commandId;
      arm.hardware->withRobot(
          [&command, &commandId, &ec](rokae::ArRobot &robot) {
            robot.moveAppend({command}, commandId, ec);
          });
      if (ec) {
        abortSdkGoal(goalHandle, result, arm, "moveAppend", ec);
        return;
      }

      // This is the call that actually triggers robot motion.
      arm.hardware->withRobot(
          [&ec](rokae::ArRobot &robot) { robot.moveStart(ec); });
      if (ec) {
        abortSdkGoal(goalHandle, result, arm, "moveStart", ec);
        return;
      }

      RCLCPP_WARN(
          get_logger(),
          "%s MoveAbsJ started: command_id=%s, joint_speed_scale=%.3f",
          arm.side.c_str(), commandId.c_str(),
          options.jointSpeedScale);

      const auto deadline =
          std::chrono::steady_clock::now() +
          std::chrono::milliseconds(
              static_cast<int>(options.timeoutS * 1000.0));
      int speedViolationCount = 0;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      while (rclcpp::ok()) {
        if (goalHandle->is_canceling()) {
          stopAndReset(arm);
          result->error_code =
              FollowJointTrajectory::Result::SUCCESSFUL;
          result->error_string = "MoveAbsJ canceled and motion reset";
          goalHandle->canceled(result);
          RCLCPP_WARN(
              get_logger(), "%s MoveAbsJ canceled", arm.side.c_str());
          return;
        }
        if (std::chrono::steady_clock::now() >= deadline) {
          stopAndReset(arm);
          abortGoal(
              goalHandle, result,
              FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED,
              "MoveAbsJ timed out and was stopped");
          return;
        }

        const auto joints = arm.hardware->withRobot(
            [&ec](rokae::ArRobot &robot) {
              return robot.jointPos(ec);
            });
        if (ec || !finiteArray(joints)) {
          stopAndReset(arm);
          const std::string message =
              ec ? sdkError("jointPos", ec)
                 : "joint feedback contains a non-finite value";
          abortGoal(
              goalHandle, result,
              FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED,
              message);
          return;
        }

        const auto velocities = arm.hardware->withRobot(
            [&ec](rokae::ArRobot &robot) {
              return robot.jointVel(ec);
            });
        if (ec || !finiteArray(velocities)) {
          stopAndReset(arm);
          const std::string message =
              ec ? sdkError("jointVel", ec)
                 : "joint velocity contains a non-finite value";
          abortGoal(
              goalHandle, result,
              FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED,
              message);
          return;
        }

        bool speedExceeded = false;
        std::size_t fastestJoint = 0;
        double fastestSpeed = 0.0;
        for (std::size_t i = 0; i < kJointCount; ++i) {
          if (joints[i] < softLimits[i][0] ||
              joints[i] > softLimits[i][1]) {
            stopAndReset(arm);
            abortGoal(
                goalHandle, result,
                FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED,
                "J" + std::to_string(i + 1) +
                    " crossed the controller soft limit");
            return;
          }
          const double speed = std::abs(velocities[i]);
          if (speed > fastestSpeed) {
            fastestSpeed = speed;
            fastestJoint = i;
          }
          if (speed > options.maxJointSpeedRadS) {
            speedExceeded = true;
          }
        }

        speedViolationCount =
            speedExceeded ? speedViolationCount + 1 : 0;
        if (speedViolationCount >= kSpeedViolationSamples) {
          stopAndReset(arm);
          abortGoal(
              goalHandle, result,
              FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED,
              "J" + std::to_string(fastestJoint + 1) +
                  " speed exceeded max_joint_speed_rad_s: " +
                  std::to_string(fastestSpeed));
          return;
        }

        publishFeedback(arm, goalHandle, target, joints);

        const auto state = arm.hardware->withRobot(
            [&ec](rokae::ArRobot &robot) {
              return robot.operationState(ec);
            });
        if (ec) {
          stopAndReset(arm);
          abortSdkGoal(goalHandle, result, arm, "operationState", ec);
          return;
        }
        if (state == rokae::OperationState::unknown) {
          stopAndReset(arm);
          abortGoal(
              goalHandle, result,
              FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED,
              "controller entered unknown operation state");
          return;
        }
        if (state == rokae::OperationState::idle) {
          double maximumError = 0.0;
          for (std::size_t i = 0; i < kJointCount; ++i) {
            maximumError =
                std::max(maximumError, std::abs(target[i] - joints[i]));
          }
          if (maximumError > options.goalToleranceRad) {
            abortGoal(
                goalHandle, result,
                FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED,
                "robot became idle before reaching the goal tolerance; "
                "maximum error=" +
                    std::to_string(maximumError));
            return;
          }

          result->error_code =
              FollowJointTrajectory::Result::SUCCESSFUL;
          result->error_string =
              "MoveAbsJ completed; command_id=" + commandId;
          goalHandle->succeed(result);
          RCLCPP_INFO(
              get_logger(), "%s MoveAbsJ completed",
              arm.side.c_str());
          return;
        }

        std::this_thread::sleep_for(kFeedbackPeriod);
      }

      stopAndReset(arm);
      abortGoal(
          goalHandle, result,
          FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED,
          "ROS 2 shutdown stopped MoveAbsJ");
    } catch (const std::exception &e) {
      stopAndReset(arm);
      abortGoal(
          goalHandle, result,
          FollowJointTrajectory::Result::INVALID_GOAL,
          std::string("MoveAbsJ action exception: ") + e.what());
    }
  }

  void publishFeedback(
      const ArmContext &arm,
      const std::shared_ptr<GoalHandle> &goalHandle,
      const std::array<double, kJointCount> &target,
      const std::array<double, kJointCount> &actual) {
    auto feedback =
        std::make_shared<FollowJointTrajectory::Feedback>();
    feedback->header.stamp = now();
    feedback->joint_names.assign(
        arm.jointNames.begin(), arm.jointNames.end());
    feedback->desired.positions.assign(target.begin(), target.end());
    feedback->actual.positions.assign(actual.begin(), actual.end());
    feedback->error.positions.resize(kJointCount);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      feedback->error.positions[i] = target[i] - actual[i];
    }
    goalHandle->publish_feedback(feedback);
  }

  void stopAndReset(ArmContext &arm) noexcept {
    try {
      std::error_code ec;
      arm.hardware->withRobot(
          [&ec](rokae::ArRobot &robot) { robot.stop(ec); });
      if (ec) {
        RCLCPP_ERROR(
            get_logger(), "%s stop failed: %s",
            arm.side.c_str(), ec.message().c_str());
      }

      const auto deadline =
          std::chrono::steady_clock::now() +
          std::chrono::seconds(5);
      while (std::chrono::steady_clock::now() < deadline) {
        ec.clear();
        const auto state = arm.hardware->withRobot(
            [&ec](rokae::ArRobot &robot) {
              return robot.operationState(ec);
            });
        if (ec || state == rokae::OperationState::idle ||
            state == rokae::OperationState::unknown) {
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }

      ec.clear();
      arm.hardware->withRobot(
          [&ec](rokae::ArRobot &robot) { robot.moveReset(ec); });
      if (ec) {
        RCLCPP_ERROR(
            get_logger(), "%s moveReset failed: %s",
            arm.side.c_str(), ec.message().c_str());
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(
          get_logger(), "%s stop/reset exception: %s",
          arm.side.c_str(), e.what());
    }
  }

  void printControllerErrors(ArmContext &arm) {
    std::error_code ec;
    const auto logs = arm.hardware->withRobot(
        [&ec](rokae::ArRobot &robot) {
          return robot.queryControllerLog(
              5, {rokae::LogInfo::error}, ec);
        });
    if (ec) return;
    for (const auto &log : logs) {
      RCLCPP_ERROR(
          get_logger(), "%s controller [%s] %s",
          arm.side.c_str(), log.timestamp.c_str(), log.content.c_str());
    }
  }

  void abortSdkGoal(
      const std::shared_ptr<GoalHandle> &goalHandle,
      const std::shared_ptr<FollowJointTrajectory::Result> &result,
      ArmContext &arm, const std::string &operation,
      const std::error_code &ec) {
    const std::string message = sdkError(operation, ec);
    printControllerErrors(arm);
    abortGoal(
        goalHandle, result,
        FollowJointTrajectory::Result::INVALID_GOAL, message);
  }

  void abortGoal(
      const std::shared_ptr<GoalHandle> &goalHandle,
      const std::shared_ptr<FollowJointTrajectory::Result> &result,
      int32_t errorCode, const std::string &message) {
    result->error_code = errorCode;
    result->error_string = message;
    goalHandle->abort(result);
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
  }

  void joinExecutionThreads() {
    std::lock_guard<std::mutex> lock(executionThreadsMutex_);
    for (auto &thread : executionThreads_) {
      if (thread.joinable()) thread.join();
    }
    executionThreads_.clear();
  }

  std::unique_ptr<ArmContext> left_;
  std::unique_ptr<ArmContext> right_;
  std::mutex executionThreadsMutex_;
  std::vector<std::thread> executionThreads_;
};

namespace rokae_driver {

std::shared_ptr<rclcpp::Node> makeMoveAbsJActionServer() {
  return std::make_shared<RokaeMoveAbsJActionServer>();
}

}  // namespace rokae_driver

#ifndef ROKAE_UNIFIED_DRIVER
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int exitCode = 0;
  try {
    auto node = std::make_shared<RokaeMoveAbsJActionServer>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    std::cerr << "MoveAbsJ action server exception: " << e.what()
              << std::endl;
    exitCode = 1;
  }
  rclcpp::shutdown();
  return exitCode;
}
#endif
