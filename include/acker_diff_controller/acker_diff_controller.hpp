// Copyright 2020 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Author: Pascal Pieper
 */

#pragma once

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include "acker_diff_controller/odometry.hpp"
#include "acker_diff_controller/speed_limiter.hpp"
#include "acker_diff_controller/visibility_control.h"
#include "../lib/minimal-PID-controller/pid.h"

namespace acker_diff_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class AckerDiffController : public controller_interface::ControllerInterface
{
	using AckermannStamped = ackermann_msgs::msg::AckermannDriveStamped;
	using Ackermann = ackermann_msgs::msg::AckermannDrive;

public:
  ACKER_DIFF_CONTROLLER_PUBLIC
  AckerDiffController();

  ACKER_DIFF_CONTROLLER_PUBLIC
  controller_interface::return_type init(const std::string & controller_name) override;

  ACKER_DIFF_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  ACKER_DIFF_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  ACKER_DIFF_CONTROLLER_PUBLIC
  controller_interface::return_type update() override;

  ACKER_DIFF_CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  ACKER_DIFF_CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ACKER_DIFF_CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ACKER_DIFF_CONTROLLER_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  ACKER_DIFF_CONTROLLER_PUBLIC
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  ACKER_DIFF_CONTROLLER_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

protected:

  typedef std::reference_wrapper<const hardware_interface::LoanedStateInterface> StateIF;
  typedef std::reference_wrapper<hardware_interface::LoanedCommandInterface>  CommandIF;

  struct WheelHandle
  {
    StateIF position;
    CommandIF velocity;
  };

  typedef StateIF SteeringHandle;


  CallbackReturn configure_side(
    const std::string & side, const std::vector<std::string> & wheel_names,
    std::vector<WheelHandle> & registered_handles);
  CallbackReturn configure_steering_angle(std::string & name,
      std::vector<SteeringHandle>& handle);

  struct WheelSpeeds {
    double left;
    double right;
  };
  typedef WheelSpeeds WheelTurningRate;

  double last_limited_linear_command_;

  WheelSpeeds calc_turning_speeds(const double linear_speed,
                                  const double turning_angle);
  WheelTurningRate mix_linear_and_angular(const AckerDiffController::WheelSpeeds& linear,
                                          const double angular_speed);
  bool update_odometry(const AckerDiffController::WheelTurningRate& set_turning_rate,
                       const rclcpp::Time& current_time);

  std::vector<std::string> left_wheel_names_;
  std::vector<std::string> right_wheel_names_;
  std::string steering_axle_name_;

  std::vector<WheelHandle> registered_left_wheel_handles_;
  std::vector<WheelHandle> registered_right_wheel_handles_;
  std::vector<SteeringHandle> registered_steering_axle_handle_;

  struct WheelParams
  {
    size_t wheels_per_side = 0;
    double separation = 0.0;  // w.r.t. the midpoint of the wheel width
    double radius = 0.0;      // Assumed to be the same for both wheels
    double separation_multiplier = 1.0;
    double wheelbase = 0;
    double left_radius_multiplier = 1.0;
    double right_radius_multiplier = 1.0;
  } wheel_params_;

  struct SteeringParams {
    bool has_position_limit = false;
    double max_angle = 0;       // rad
    double min_angle = 0;       // rad
    double max_angular_velocity = 0;    // rad/s
    double max_angular_acceleration = 0;// rad/s^2
    double limit_linear_at_angle_diff_to_setpoint = 0; // rad
  } steering_params_;

  struct OdometryParams
  {
    bool open_loop = false;
    bool enable_odom_tf = true;
    std::string base_frame_id = "base_link";
    std::string odom_frame_id = "odom";
    std::array<double, 6> pose_covariance_diagonal;
    std::array<double, 6> twist_covariance_diagonal;
  } odom_params_;

  PID::Settings pid_params_ = PID::Settings{
    .Kp = 1, .Ki = 0.5, .Kd = 0.01,
    .max = NAN, .min = NAN,
    .max_dv = NAN, .overshoot_integral_adaptation = NAN
  };
  PID pid_controller_;

  Odometry odometry_;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
    realtime_odometry_publisher_ = nullptr;

  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_ =
    nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
    realtime_odometry_transform_publisher_ = nullptr;

  // Timeout to consider cmd_vel commands old
  std::chrono::milliseconds cmd_vel_timeout_{500};

  bool subscriber_is_active_ = false;
  rclcpp::Subscription<AckermannStamped>::SharedPtr command_subscriber_ = nullptr;
  rclcpp::Subscription<Ackermann>::SharedPtr command_unstamped_subscriber_ = nullptr;

  realtime_tools::RealtimeBox<std::shared_ptr<AckermannStamped>> received_command_msg_ptr_{nullptr};

  std::queue<AckermannStamped> previous_commands_;  // last two commands

  // speed limiters
  SpeedLimiter limiter_linear_;
  SpeedLimiter limiter_angle_;

  bool publish_limited_velocity_ = false;
  std::shared_ptr<rclcpp::Publisher<AckermannStamped>> limited_command_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<AckermannStamped>> realtime_limited_command_publisher_ =
    nullptr;

  rclcpp::Clock clock_{RCL_STEADY_TIME};
  rclcpp::Time previous_update_timestamp_{0};

  bool is_halted = false;
  bool use_stamped_vel_ = true;

  bool reset();
  void halt();
};
}  // namespace acker_diff_controller
