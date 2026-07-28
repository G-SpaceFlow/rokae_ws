/**
 * @file ros_movel_service.cpp
 * @brief ROS 2 services backed by the xCoreSDK MoveL command.
 *
 * Services:
 *   /left_arm/move_l
 *   /right_arm/move_l
 *   /left_arm/move_l_relative
 *   /right_arm/move_l_relative
 *   /left_arm/move_l_target
 *   /right_arm/move_l_target
 *   /left_arm/get_cartesian_state
 *   /right_arm/get_cartesian_state
 *
 * The request pose is [x, y, z, rx, ry, rz] in the robot's configured
 * external-reference frame. Translation is in metres; XYZ Euler angles and
 * the elbow value are in radians. The callback returns only after motion stops,
 * fails, or reaches its timeout.
 */

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rokae_interfaces/srv/get_cartesian_state.hpp"
#include "rokae_interfaces/srv/move_l.hpp"
#include "rokae_interfaces/srv/move_l_relative.hpp"
#include "rokae_interfaces/srv/move_l_target.hpp"
#include "rokae/robot.h"

namespace {

using GetCartesianState = rokae_interfaces::srv::GetCartesianState;
using MoveL = rokae_interfaces::srv::MoveL;
using MoveLRelative = rokae_interfaces::srv::MoveLRelative;
using MoveLTarget = rokae_interfaces::srv::MoveLTarget;
constexpr auto kPollPeriod = std::chrono::milliseconds(100);
constexpr auto kMotionStartGracePeriod = std::chrono::milliseconds(100);

struct PoseError {
  double translationM{0.0};
  double rotationRad{0.0};
};

std::string sdkError(
    const std::string &operation, const std::error_code &ec) {
  return operation + " failed: code=" + std::to_string(ec.value()) +
         ", message=" + ec.message();
}

struct ArmContext {
  std::string side;
  std::string parameterPrefix;
  std::unique_ptr<rokae::ArRobot> robot;
  std::mutex mutex;
  rclcpp::CallbackGroup::SharedPtr callbackGroup;
  rclcpp::Service<MoveL>::SharedPtr service;
  rclcpp::Service<MoveLRelative>::SharedPtr relativeService;
  rclcpp::Service<MoveLTarget>::SharedPtr targetService;
  rclcpp::Service<GetCartesianState>::SharedPtr stateService;
};

}  // namespace

class RokaeMoveLService : public rclcpp::Node {
 public:
  RokaeMoveLService() : Node("rokae_movel_service") {
    left_ = createArm(
        "left", "192.168.0.160", "192.168.0.10",
        "/left_arm/move_l", "/left_arm/move_l_relative");
    right_ = createArm(
        "right", "192.168.2.160", "192.168.2.10",
        "/right_arm/move_l", "/right_arm/move_l_relative");

    RCLCPP_WARN(
        get_logger(),
        "MoveL services are ready. This node does not power on either robot. "
        "Use a small first move, low speed, a clear workspace and an "
        "accessible E-stop.");
  }

 private:
  std::unique_ptr<ArmContext> createArm(
      const std::string &side, const std::string &defaultRobotIp,
      const std::string &defaultLocalIp, const std::string &serviceName,
      const std::string &relativeServiceName) {
    const std::string prefix = side + "_arm";
    const auto robotIp = declare_parameter<std::string>(
        prefix + ".robot_ip", defaultRobotIp);
    const auto localIp = declare_parameter<std::string>(
        prefix + ".local_ip", defaultLocalIp);
    declare_parameter<double>(prefix + ".timeout_s", 60.0);
    declare_parameter<double>(prefix + ".max_translation_delta_m", 0.10);
    declare_parameter<double>(prefix + ".max_rotation_delta_rad", 0.35);
    declare_parameter<double>(prefix + ".max_speed_mm_s", 250.0);
    declare_parameter<double>(prefix + ".max_zone_mm", 20.0);
    declare_parameter<double>(prefix + ".motion_start_timeout_s", 3.0);
    declare_parameter<double>(prefix + ".goal_position_tolerance_m", 0.002);
    declare_parameter<double>(
        prefix + ".goal_rotation_tolerance_rad", 0.02);

    auto arm = std::make_unique<ArmContext>();
    arm->side = side;
    arm->parameterPrefix = prefix;

    RCLCPP_INFO(
        get_logger(), "Connecting %s arm: robot=%s, local=%s",
        side.c_str(), robotIp.c_str(), localIp.c_str());
    arm->robot = std::make_unique<rokae::ArRobot>(robotIp, localIp);
    arm->callbackGroup = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    ArmContext *const armPtr = arm.get();
    arm->service = create_service<MoveL>(
        serviceName,
        [this, armPtr](
            const std::shared_ptr<MoveL::Request> request,
            std::shared_ptr<MoveL::Response> response) {
          execute(*armPtr, *request, *response);
        },
        rmw_qos_profile_services_default,
        arm->callbackGroup);
    RCLCPP_INFO(get_logger(), "%s is ready", serviceName.c_str());

    arm->relativeService = create_service<MoveLRelative>(
        relativeServiceName,
        [this, armPtr](
            const std::shared_ptr<MoveLRelative::Request> request,
            std::shared_ptr<MoveLRelative::Response> response) {
          executeRelative(*armPtr, *request, *response);
        },
        rmw_qos_profile_services_default,
        arm->callbackGroup);
    RCLCPP_INFO(
        get_logger(), "%s is ready", relativeServiceName.c_str());

    const std::string targetServiceName =
        "/" + side + "_arm/move_l_target";
    arm->targetService = create_service<MoveLTarget>(
        targetServiceName,
        [this, armPtr](
            const std::shared_ptr<MoveLTarget::Request> request,
            std::shared_ptr<MoveLTarget::Response> response) {
          executePoseTarget(*armPtr, *request, *response);
        },
        rmw_qos_profile_services_default,
        arm->callbackGroup);
    RCLCPP_INFO(
        get_logger(), "%s is ready", targetServiceName.c_str());

    const std::string stateServiceName =
        "/" + side + "_arm/get_cartesian_state";
    arm->stateService = create_service<GetCartesianState>(
        stateServiceName,
        [this, armPtr](
            const std::shared_ptr<GetCartesianState::Request>,
            std::shared_ptr<GetCartesianState::Response> response) {
          readCartesianState(*armPtr, *response);
        },
        rmw_qos_profile_services_default,
        arm->callbackGroup);
    RCLCPP_INFO(
        get_logger(), "%s is ready", stateServiceName.c_str());
    return arm;
  }

  void readCartesianState(
      ArmContext &arm, GetCartesianState::Response &response) {
    std::unique_lock<std::mutex> lock(arm.mutex, std::try_to_lock);
    if (!lock.owns_lock()) {
      fail(
          response,
          arm.side + " arm is already executing a MoveL request");
      return;
    }

    std::error_code ec;
    const auto current =
        arm.robot->cartPosture(rokae::CoordinateType::endInRef, ec);
    if (ec) {
      fail(response, sdkError("cartPosture(endInRef)", ec));
      return;
    }
    for (std::size_t index = 0; index < 3; ++index) {
      response.pose[index] = current.trans[index];
      response.pose[index + 3] = current.rpy[index];
    }
    response.success = true;
    response.message = "current Cartesian state";
  }

  void executePoseTarget(
      ArmContext &arm, const MoveLTarget::Request &request,
      MoveLTarget::Response &response) {
    std::unique_lock<std::mutex> lock(arm.mutex, std::try_to_lock);
    if (!lock.owns_lock()) {
      fail(response, arm.side + " arm is already executing a MoveL request");
      return;
    }

    const bool finitePosition = std::all_of(
        request.position.begin(), request.position.end(),
        [](double value) { return std::isfinite(value); });
    const bool finiteOrientation = std::all_of(
        request.orientation_rpy.begin(), request.orientation_rpy.end(),
        [](double value) { return std::isfinite(value); });
    if (!finitePosition || !finiteOrientation) {
      fail(response, "target position and orientation must be finite");
      return;
    }

    std::string reason;
    if (!validateMotionOptions(
            arm, request.speed_mm_s, request.zone_mm, reason)) {
      fail(response, reason);
      return;
    }

    try {
      std::error_code ec;
      const auto current =
          arm.robot->cartPosture(rokae::CoordinateType::endInRef, ec);
      if (ec) {
        fail(response, sdkError("cartPosture(endInRef)", ec));
        return;
      }

      // Preserve the controller-provided elbow, configuration and external
      // axes. Vision only replaces the absolute TCP position and requested
      // absolute XYZ Euler orientation axes.
      rokae::CartesianPosition target = current;
      for (std::size_t index = 0; index < 3; ++index) {
        target.trans[index] = request.position[index];
        if (request.orientation_override[index]) {
          target.rpy[index] = request.orientation_rpy[index];
        }
      }

      if (!validateDelta(arm, current, target, reason)) {
        fail(response, reason);
        return;
      }
      executeTarget(
          arm, target, request.speed_mm_s, request.zone_mm, response);
    } catch (const std::exception &exception) {
      fail(
          response,
          std::string("pose-target MoveL exception: ") +
              exception.what());
    }
  }

  void executeRelative(
      ArmContext &arm, const MoveLRelative::Request &request,
      MoveLRelative::Response &response) {
    std::unique_lock<std::mutex> lock(arm.mutex, std::try_to_lock);
    if (!lock.owns_lock()) {
      fail(response, arm.side + " arm is already executing a MoveL request");
      return;
    }

    const bool finiteTranslation = std::all_of(
        request.translation.begin(), request.translation.end(),
        [](double value) { return std::isfinite(value); });
    const bool finiteOrientation = std::all_of(
        request.orientation_rpy.begin(), request.orientation_rpy.end(),
        [](double value) { return std::isfinite(value); });
    if (!finiteTranslation || !finiteOrientation) {
      fail(
          response,
          "relative translation and absolute orientation must be finite");
      return;
    }

    std::string reason;
    if (!validateMotionOptions(
            arm, request.speed_mm_s, request.zone_mm, reason)) {
      fail(response, reason);
      return;
    }

    try {
      std::error_code ec;
      const auto current =
          arm.robot->cartPosture(rokae::CoordinateType::endInRef, ec);
      if (ec) {
        fail(response, sdkError("cartPosture(endInRef)", ec));
        return;
      }

      // Keep the complete SDK CartesianPosition, including its elbow,
      // configuration and external-axis data. AR controllers may report
      // hasElbow=false even though this copied position is a valid MoveL seed.
      rokae::CartesianPosition target = current;
      for (std::size_t i = 0; i < 3; ++i) {
        target.trans[i] += request.translation[i];
        if (request.orientation_override[i]) {
          target.rpy[i] = request.orientation_rpy[i];
        }
      }

      if (!validateDelta(arm, current, target, reason)) {
        fail(response, reason);
        return;
      }
      executeTarget(
          arm, target, request.speed_mm_s, request.zone_mm, response);
    } catch (const std::exception &exception) {
      fail(
          response,
          std::string("relative MoveL exception: ") + exception.what());
    }
  }

  void execute(
      ArmContext &arm, const MoveL::Request &request,
      MoveL::Response &response) {
    std::unique_lock<std::mutex> lock(arm.mutex, std::try_to_lock);
    if (!lock.owns_lock()) {
      fail(response, arm.side + " arm is already executing a MoveL request");
      return;
    }

    std::string reason;
    if (!validateRequest(arm, request, reason)) {
      fail(response, reason);
      return;
    }

    try {
      std::error_code ec;
      const auto current =
          arm.robot->cartPosture(rokae::CoordinateType::endInRef, ec);
      if (ec) {
        fail(response, sdkError("cartPosture(endInRef)", ec));
        return;
      }

      rokae::CartesianPosition target({
          request.pose[0], request.pose[1], request.pose[2],
          request.pose[3], request.pose[4], request.pose[5]});
      target.elbow = request.elbow;
      target.hasElbow = true;
      if (!validateDelta(arm, current, target, reason)) {
        fail(response, reason);
        return;
      }
      executeTarget(
          arm, target, request.speed_mm_s, request.zone_mm, response);
    } catch (const std::exception &exception) {
      fail(response, std::string("MoveL exception: ") + exception.what());
    }
  }

  template <typename ResponseT>
  void executeTarget(
      ArmContext &arm, const rokae::CartesianPosition &target,
      double speedMmS, double zoneMm, ResponseT &response) {
    const double timeoutS = get_parameter(
        arm.parameterPrefix + ".timeout_s").as_double();
    const double motionStartTimeoutS = get_parameter(
        arm.parameterPrefix + ".motion_start_timeout_s").as_double();
    const double positionToleranceM = get_parameter(
        arm.parameterPrefix + ".goal_position_tolerance_m").as_double();
    const double rotationToleranceRad = get_parameter(
        arm.parameterPrefix + ".goal_rotation_tolerance_rad").as_double();

    std::string reason;
    if (!validateMonitorOptions(
            timeoutS, motionStartTimeoutS, positionToleranceM,
            rotationToleranceRad, reason)) {
      fail(response, reason);
      return;
    }

    std::error_code ec;
    arm.robot->setMotionControlMode(
        rokae::MotionControlMode::NrtCommand, ec);
    if (ec) {
      fail(response, sdkError("setMotionControlMode(NrtCommand)", ec));
      return;
    }
    arm.robot->setOperateMode(rokae::OperateMode::automatic, ec);
    if (ec) {
      fail(response, sdkError("setOperateMode(automatic)", ec));
      return;
    }
    arm.robot->setDefaultConfOpt(false, ec);
    if (ec) {
      fail(response, sdkError("setDefaultConfOpt(false)", ec));
      return;
    }
    arm.robot->moveReset(ec);
    if (ec) {
      fail(response, sdkError("moveReset", ec));
      return;
    }

    const rokae::MoveLCommand command(target, speedMmS, zoneMm);
    std::string commandId;
    arm.robot->moveAppend({command}, commandId, ec);
    if (ec) {
      fail(response, sdkError("moveAppend(MoveL)", ec));
      return;
    }
    arm.robot->moveStart(ec);
    if (ec) {
      fail(response, sdkError("moveStart", ec));
      return;
    }

    const auto commandStart = std::chrono::steady_clock::now();
    const auto deadline = commandStart +
        std::chrono::duration<double>(timeoutS);
    const auto motionStartDeadline = commandStart +
        std::chrono::duration<double>(motionStartTimeoutS);
    bool motionObserved = false;

    // The controller can still report the previous idle state immediately
    // after moveStart(). The known-good SDK example waits before polling.
    std::this_thread::sleep_for(kMotionStartGracePeriod);

    const auto complete = [&](const PoseError &error) {
      response.success = true;
      response.message =
          "MoveL completed; command_id=" + commandId +
          "; position_error_m=" + std::to_string(error.translationM) +
          "; rotation_error_rad=" + std::to_string(error.rotationRad);
      RCLCPP_INFO(
          get_logger(),
          "%s arm MoveL completed: position_error=%.6f m, "
          "rotation_error=%.6f rad",
          arm.side.c_str(), error.translationM, error.rotationRad);
    };

    while (rclcpp::ok()) {
      const auto state = arm.robot->operationState(ec);
      if (ec) {
        stopAndReset(arm);
        fail(response, sdkError("operationState", ec));
        return;
      }
      if (state == rokae::OperationState::unknown) {
        stopAndReset(arm);
        fail(response, "robot operation state became unknown");
        return;
      }

      const auto now = std::chrono::steady_clock::now();
      if (state != rokae::OperationState::idle) {
        motionObserved = true;
      } else {
        PoseError goalError;
        if (!readGoalError(arm, target, goalError, reason)) {
          stopAndReset(arm);
          fail(response, reason);
          return;
        }
        const bool targetReached =
            goalError.translationM <= positionToleranceM &&
            goalError.rotationRad <= rotationToleranceRad;
        if (targetReached) {
          complete(goalError);
          return;
        }
        if (motionObserved) {
          stopAndReset(arm);
          fail(
              response,
              "robot became idle before reaching the MoveL target; "
              "position_error_m=" +
                  std::to_string(goalError.translationM) +
                  ", rotation_error_rad=" +
                  std::to_string(goalError.rotationRad));
          return;
        }
        if (now >= motionStartDeadline) {
          stopAndReset(arm);
          fail(
              response,
              "MoveL did not enter a running state within " +
                  std::to_string(motionStartTimeoutS) +
                  " s; position_error_m=" +
                  std::to_string(goalError.translationM) +
                  ", rotation_error_rad=" +
                  std::to_string(goalError.rotationRad));
          return;
        }
      }

      if (now >= deadline) {
        stopAndReset(arm);
        fail(
            response,
            "MoveL timed out after " + std::to_string(timeoutS) +
                " s; motion stopped and reset");
        return;
      }
      std::this_thread::sleep_for(kPollPeriod);
    }
    stopAndReset(arm);
    fail(response, "ROS shutdown stopped and reset MoveL");
  }

  bool validateRequest(
      const ArmContext &arm, const MoveL::Request &request,
      std::string &reason) const {
    const bool finitePose = std::all_of(
        request.pose.begin(), request.pose.end(),
        [](double value) { return std::isfinite(value); });
    if (!finitePose || !std::isfinite(request.elbow)) {
      reason = "pose and elbow must be finite";
      return false;
    }
    return validateMotionOptions(
        arm, request.speed_mm_s, request.zone_mm, reason);
  }

  bool validateMotionOptions(
      const ArmContext &arm, double speedMmS, double zoneMm,
      std::string &reason) const {
    if (!std::isfinite(speedMmS) || !std::isfinite(zoneMm)) {
      reason = "speed_mm_s and zone_mm must be finite";
      return false;
    }
    const double maxSpeed = get_parameter(
        arm.parameterPrefix + ".max_speed_mm_s").as_double();
    const double maxZone = get_parameter(
        arm.parameterPrefix + ".max_zone_mm").as_double();
    if (speedMmS <= 0.0 || speedMmS > maxSpeed) {
      reason = "speed_mm_s must be in (0, " +
          std::to_string(maxSpeed) + "]";
      return false;
    }
    if (zoneMm < 0.0 || zoneMm > maxZone) {
      reason = "zone_mm must be in [0, " + std::to_string(maxZone) + "]";
      return false;
    }
    return true;
  }

  bool validateMonitorOptions(
      double timeoutS, double motionStartTimeoutS,
      double positionToleranceM, double rotationToleranceRad,
      std::string &reason) const {
    const std::array<double, 4> values{
        timeoutS, motionStartTimeoutS, positionToleranceM,
        rotationToleranceRad};
    if (!std::all_of(
            values.begin(), values.end(),
            [](double value) { return std::isfinite(value); })) {
      reason = "MoveL timeout and goal tolerances must be finite";
      return false;
    }
    if (timeoutS < 1.0 || timeoutS > 300.0) {
      reason = "timeout_s must be in [1, 300]";
      return false;
    }
    if (motionStartTimeoutS <= 0.0 ||
        motionStartTimeoutS > std::min(timeoutS, 10.0)) {
      reason =
          "motion_start_timeout_s must be in (0, min(timeout_s, 10)]";
      return false;
    }
    if (positionToleranceM <= 0.0 || positionToleranceM > 0.05) {
      reason = "goal_position_tolerance_m must be in (0, 0.05]";
      return false;
    }
    if (rotationToleranceRad <= 0.0 || rotationToleranceRad > 0.20) {
      reason = "goal_rotation_tolerance_rad must be in (0, 0.20]";
      return false;
    }
    return true;
  }

  PoseError calculatePoseError(
      const rokae::CartesianPosition &current,
      const rokae::CartesianPosition &target) const {
    double translationSquared = 0.0;
    double rotationSquared = 0.0;
    for (std::size_t i = 0; i < 3; ++i) {
      const double translationDelta =
          target.trans[i] - current.trans[i];
      translationSquared += translationDelta * translationDelta;
      const double rotationDelta = std::remainder(
          target.rpy[i] - current.rpy[i], 2.0 * M_PI);
      rotationSquared += rotationDelta * rotationDelta;
    }
    return {
        std::sqrt(translationSquared),
        std::sqrt(rotationSquared),
    };
  }

  bool readGoalError(
      ArmContext &arm, const rokae::CartesianPosition &target,
      PoseError &error, std::string &reason) {
    std::error_code ec;
    const auto current =
        arm.robot->cartPosture(rokae::CoordinateType::endInRef, ec);
    if (ec) {
      reason = sdkError("cartPosture(endInRef)", ec);
      return false;
    }
    error = calculatePoseError(current, target);
    return true;
  }

  void stopAndReset(ArmContext &arm) {
    std::error_code ec;
    arm.robot->stop(ec);
    if (ec) {
      RCLCPP_ERROR(
          get_logger(), "%s arm stop failed: %s",
          arm.side.c_str(), sdkError("stop", ec).c_str());
    }
    ec.clear();
    arm.robot->moveReset(ec);
    if (ec) {
      RCLCPP_ERROR(
          get_logger(), "%s arm moveReset failed: %s",
          arm.side.c_str(), sdkError("moveReset", ec).c_str());
    }
  }

  bool validateDelta(
      const ArmContext &arm, const rokae::CartesianPosition &current,
      const rokae::CartesianPosition &target, std::string &reason) const {
    const PoseError error = calculatePoseError(current, target);
    const double maxTranslation = get_parameter(
        arm.parameterPrefix + ".max_translation_delta_m").as_double();
    const double maxRotation = get_parameter(
        arm.parameterPrefix + ".max_rotation_delta_rad").as_double();
    if (error.translationM > maxTranslation) {
      reason =
          "translation delta " + std::to_string(error.translationM) +
          " m exceeds max_translation_delta_m=" +
          std::to_string(maxTranslation);
      return false;
    }
    if (error.rotationRad > maxRotation) {
      reason =
          "XYZ Euler-angle delta " + std::to_string(error.rotationRad) +
          " rad exceeds max_rotation_delta_rad=" +
          std::to_string(maxRotation);
      return false;
    }
    return true;
  }

  template <typename ResponseT>
  void fail(ResponseT &response, const std::string &message) {
    response.success = false;
    response.message = message;
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
  }

  std::unique_ptr<ArmContext> left_;
  std::unique_ptr<ArmContext> right_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<RokaeMoveLService>();
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception &exception) {
    RCLCPP_FATAL(
        rclcpp::get_logger("rokae_movel_service"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
