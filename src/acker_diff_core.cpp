// Copyright 2022
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
 * Moved out of acker_diff_controller.cpp unchanged in behaviour.
 */

#include "acker_diff_controller/acker_diff_core.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace acker_diff_controller
{

namespace
{
// TODO: magic value where turning angle is so small that different speeds are not needed
constexpr double kMinAbsTurningAngleForAckermann = 0.05;
}  // namespace

WheelPair calc_turning_speeds(
  const WheelGeometry & geometry, const double linear_speed, const double turning_angle)
{
  WheelPair ret{linear_speed, linear_speed};
  if (turning_angle == 0) {
    return ret;
  }

  const double wheel_separation = geometry.effective_separation();
  const double wheel_base_distance = geometry.wheelbase;

  auto abs_alpha = M_PI_2 - std::abs(turning_angle);
  auto turning_radius = wheel_base_distance / cos(abs_alpha);

  ret.left = (turning_radius + wheel_separation / 2.0) * atan(linear_speed / turning_radius);
  ret.right = (turning_radius - wheel_separation / 2.0) * atan(linear_speed / turning_radius);

  if (turning_angle < 0) {
    std::swap(ret.left, ret.right);
  }
  return ret;
}

WheelPair mix_linear_and_angular(
  const WheelGeometry & geometry, const WheelPair & linear, const double angular_speed)
{
  const double wheel_separation = geometry.effective_separation();
  return WheelPair{
    (linear.left - angular_speed * wheel_separation / 2.0) / geometry.left_radius(),
    (linear.right + angular_speed * wheel_separation / 2.0) / geometry.right_radius()
  };
}

Twist2D open_loop_twist(const WheelGeometry & geometry, const WheelPair & turning_rate)
{
  // back-calculate wheel turning rates to linear/angular proportions
  const WheelPair set_velocity{
    turning_rate.left * geometry.left_radius(),
    turning_rate.right * geometry.right_radius()};

  // basically inverse of mix_linear_and_angular()
  const double mean_linear = (set_velocity.left + set_velocity.right) / 2;
  const double angular_proportion =
    (set_velocity.right - set_velocity.left) / geometry.effective_separation();  // Why not divide by 2?
  return Twist2D{mean_linear, angular_proportion};
}

Pose2D front_axle_to_rear_axle_centre(
  const Pose2D & front_axle, const double steering_angle, const double wheelbase)
{
  const double chassis_heading = front_axle.heading + steering_angle;
  return Pose2D{
    front_axle.x - wheelbase * std::cos(chassis_heading),
    front_axle.y - wheelbase * std::sin(chassis_heading),
    chassis_heading};
}

PlanarTwist front_axle_twist_to_rear_axle_centre(
  const double front_linear, const double front_angular, const double steering_angle,
  const double steering_rate, const double wheelbase)
{
  const double chassis_angular = front_angular + steering_rate;
  // The pivot moves along the front axle's heading, which is turned by
  // -steering_angle against the chassis. The rear axle centre sits at
  // (-wheelbase, 0) from the pivot, so the rotation adds -angular * wheelbase in y.
  return PlanarTwist{
    front_linear * std::cos(steering_angle),
    -front_linear * std::sin(steering_angle) - chassis_angular * wheelbase,
    chassis_angular};
}

void AckerDiffCore::configure(const Config & config)
{
  config_ = config;
  odometry_.setWheelParams(
    config_.geometry.effective_separation(),
    config_.geometry.left_radius(),
    config_.geometry.right_radius());
  odometry_.setVelocityRollingWindowSize(config_.velocity_rolling_window_size);
  reset();
}

void AckerDiffCore::reset()
{
  odometry_.resetOdometry();
  pid_controller_ = PID();
  previous_commands_ = {AckerCommand{}, AckerCommand{}};
  last_limited_linear_command_ = 0;
  has_last_steering_angle_ = false;
  rear_axle_centre_ = BaseState{};
  rear_axle_centre_.pose = front_axle_to_rear_axle_centre(
    Pose2D{0.0, 0.0, 0.0}, 0.0, config_.geometry.wheelbase);
}

void AckerDiffCore::update_rear_axle_centre(const double steering_angle, const double now)
{
  double steering_rate = 0.0;
  if (has_last_steering_angle_ && now > last_steering_time_) {
    steering_rate = (steering_angle - last_steering_angle_) / (now - last_steering_time_);
  }
  has_last_steering_angle_ = true;
  last_steering_angle_ = steering_angle;
  last_steering_time_ = now;

  const double wheelbase = config_.geometry.wheelbase;
  rear_axle_centre_.pose = front_axle_to_rear_axle_centre(
    Pose2D{odometry_.getX(), odometry_.getY(), odometry_.getHeading()},
    steering_angle, wheelbase);
  rear_axle_centre_.twist = front_axle_twist_to_rear_axle_centre(
    odometry_.getLinear(), odometry_.getAngular(), steering_angle, steering_rate, wheelbase);
}

void AckerDiffCore::restart_after_invalid_input()
{
  // The wheels are halted; resume from standstill with a fresh steering loop.
  pid_controller_ = PID();
  last_limited_linear_command_ = 0;
}

void AckerDiffCore::halt()
{
  last_limited_linear_command_ = 0;
}

AckerDiffCore::Output AckerDiffCore::step(
  const AckerCommand & command, const Measurement & measurement, const double dt,
  const double now)
{
  Output out;

  // linear_command and angle_command may be limited further by SpeedLimit,
  double linear_command = command.speed;
  double angle_command = command.steering_angle;
  // angular is considered in the direction of steering error
  const double angular_command = std::abs(command.steering_angle_velocity);

  // The limiters compare against the previous *unlimited* commands.
  const AckerCommand last_command = previous_commands_[1];
  const AckerCommand second_to_last_command = previous_commands_[0];

  config_.linear_limiter.limit(
    linear_command, last_command.speed, second_to_last_command.speed, dt);
  config_.angle_limiter.limit(
    angle_command, last_command.steering_angle, second_to_last_command.steering_angle, dt);

  previous_commands_ = {last_command, command};

  out.limited_command.steering_angle = angle_command;
  out.limited_command.steering_angle_velocity = angular_command;

  const double current_steering_angle = measurement.steering_angle;
  if (std::isnan(current_steering_angle)) {
    out.status = Status::invalid_steering_angle;
    restart_after_invalid_input();
    return out;
  }

  double angular_correction = 0;
  if (angular_command > 0) {
    // only regulate when allowed angular velocity is > 0
    if (last_command.steering_angle_velocity == 0) {
      // we transitioned from passive to active steering, reset PID
      pid_controller_ = PID();
    }
    auto current_pid_params = config_.pid;  // local copy to modify
    // this limits the max angular correction to the requested steering_angle_velocity
    current_pid_params.max = std::min(config_.pid.max, angular_command);
    current_pid_params.min = -current_pid_params.max;

    // negative feedback, because sensor is not mathematically but logically oriented
    angular_correction =
      -pid_controller_.calculate(angle_command, current_steering_angle, dt, current_pid_params);

    // check for significant controller overshoot on maximum angle for safety
    if ((current_steering_angle >= config_.steering.max_angle && angular_correction < 0) ||
      (current_steering_angle <= config_.steering.min_angle && angular_correction > 0))
    {
      // we would oversteer into endstops
      angular_correction = 0;
      pid_controller_ = PID();  // resets internals
      out.steering_limit_reset = true;
    }
  }
  out.angular_correction = angular_correction;

  // reduce linear speed if the steering error is too big
  if (std::abs(angular_correction) > config_.steering.limit_linear_at_angle_diff_to_setpoint) {
    linear_command = 0;
    out.linear_stopped = true;
  }

  // limit linear acceleration
  config_.linear_limiter.limit_velocity(linear_command);  // only needed if logic earlier would also increase speed
  config_.linear_limiter.limit_acceleration(linear_command, last_limited_linear_command_, dt);

  last_limited_linear_command_ = linear_command;
  out.limited_command.speed = linear_command;

  WheelPair linear_speed{linear_command, linear_command};
  if (std::abs(angle_command) > kMinAbsTurningAngleForAckermann) {
    linear_speed = calc_turning_speeds(config_.geometry, linear_command, angle_command);
  }

  const WheelPair final_turning_rate =
    mix_linear_and_angular(config_.geometry, linear_speed, angular_correction);

  if (config_.open_loop) {
    const Twist2D twist = open_loop_twist(config_.geometry, final_turning_rate);
    odometry_.updateOpenLoop(twist.linear, twist.angular, now);
  } else {
    const WheelPair & position = measurement.wheel_position;
    if (std::isnan(position.left) || std::isnan(position.right)) {
      out.status = Status::invalid_wheel_position;
      update_rear_axle_centre(current_steering_angle, now);
      restart_after_invalid_input();
      return out;
    }
    odometry_.update(position.left, position.right, now);
  }
  update_rear_axle_centre(current_steering_angle, now);

  out.wheel_rates = final_turning_rate;
  return out;
}

}  // namespace acker_diff_controller
