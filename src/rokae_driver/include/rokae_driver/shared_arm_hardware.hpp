#ifndef ROKAE_DRIVER__SHARED_ARM_HARDWARE_HPP_
#define ROKAE_DRIVER__SHARED_ARM_HARDWARE_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include "rokae/robot.h"

namespace rokae_driver {

class SharedArmHardware {
 public:
  SharedArmHardware(
      std::string side, std::string robot_ip, std::string local_ip)
      : side_(std::move(side)),
        robot_ip_(std::move(robot_ip)),
        local_ip_(std::move(local_ip)),
        robot_(std::make_unique<rokae::ArRobot>(robot_ip_, local_ip_)) {}

  SharedArmHardware(const SharedArmHardware &) = delete;
  SharedArmHardware &operator=(const SharedArmHardware &) = delete;

  const std::string &side() const noexcept { return side_; }
  const std::string &robotIp() const noexcept { return robot_ip_; }
  const std::string &localIp() const noexcept { return local_ip_; }

  // Serializes commands that change an arm. Read-only state access does not
  // take this lock, so state publishing can continue during motion.
  std::mutex &commandMutex() noexcept { return command_mutex_; }

  // xCoreSDK does not document ArRobot as thread-safe. Every access to the
  // shared instance is serialized through this helper.
  template <typename Callable>
  decltype(auto) withRobot(Callable &&callable) {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    return std::forward<Callable>(callable)(*robot_);
  }

  // Serializes operations performed through an RtMotionControl handle with
  // all other accesses to this ArRobot instance.
  template <typename Callable>
  decltype(auto) withSdkLock(Callable &&callable) {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    return std::forward<Callable>(callable)();
  }

 private:
  std::string side_;
  std::string robot_ip_;
  std::string local_ip_;
  std::unique_ptr<rokae::ArRobot> robot_;
  std::mutex sdk_mutex_;
  std::mutex command_mutex_;
};

class SharedArmRegistry {
 public:
  static SharedArmRegistry &instance() {
    static SharedArmRegistry registry;
    return registry;
  }

  std::shared_ptr<SharedArmHardware> getOrCreate(
      const std::string &side, const std::string &robot_ip,
      const std::string &local_ip) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto existing = arms_.find(side);
    if (existing != arms_.end()) {
      if (existing->second->robotIp() != robot_ip ||
          existing->second->localIp() != local_ip) {
        throw std::invalid_argument(
            "inconsistent " + side + " arm network parameters: first=" +
            existing->second->robotIp() + "/" +
            existing->second->localIp() + ", requested=" + robot_ip +
            "/" + local_ip);
      }
      return existing->second;
    }

    auto hardware = std::make_shared<SharedArmHardware>(
        side, robot_ip, local_ip);
    arms_.emplace(side, hardware);
    return hardware;
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    arms_.clear();
  }

 private:
  SharedArmRegistry() = default;

  std::mutex mutex_;
  std::map<std::string, std::shared_ptr<SharedArmHardware>> arms_;
};

inline std::shared_ptr<SharedArmHardware> sharedArm(
    const std::string &side, const std::string &robot_ip,
    const std::string &local_ip) {
  return SharedArmRegistry::instance().getOrCreate(
      side, robot_ip, local_ip);
}

}  // namespace rokae_driver

#endif  // ROKAE_DRIVER__SHARED_ARM_HARDWARE_HPP_
