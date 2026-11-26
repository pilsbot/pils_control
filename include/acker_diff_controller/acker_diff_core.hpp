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
 * Acker-diff steering core: everything the controller computes, free of ROS.
 *
 * The controller is a thin adapter around this module. It reads parameters and
 * hardware interfaces, handles the command timeout, and publishes results.
 * Keeping this module ROS-free lets it build and test identically on the Foxy
 * and Humble lines (docs/adr/0001 in the workspace repository).
 */

#pragma once

#include <array>
#include <cstddef>

#include "acker_diff_controller/odometry.hpp"
#include "acker_diff_controller/speed_limiter.hpp"
#include "pid.h"

namespace acker_diff_controller
{

struct WheelGeometry
{
  double separation = 0.0;  // w.r.t. the midpoint of the wheel width
  double radius = 0.0;      // Assumed to be the same for both wheels
  double wheelbase = 0.0;   // front axle pivot to rear axle
  double separation_multiplier = 1.0;
  double left_radius_multiplier = 1.0;
  double right_radius_multiplier = 1.0;

  double effective_separation() const { return separation_multiplier * separation; }
  double left_radius() const { return left_radius_multiplier * radius; }
  double right_radius() const { return right_radius_multiplier * radius; }
};

struct SteeringParams
{
  bool has_position_limit = false;
  double max_angle = 0;                // rad
  double min_angle = 0;                // rad
  double max_angular_velocity = 0;     // rad/s
  double max_angular_acceleration = 0; // rad/s^2
  double limit_linear_at_angle_diff_to_setpoint = 0;  // rad
};

struct WheelPair
{
  double left;
  double right;
};

struct Twist2D
{
  double linear;
  double angular;
};

// Speed of each front wheel [m/s] for driving at linear_speed with the given steering angle.
WheelPair calc_turning_speeds(
  const WheelGeometry & geometry, double linear_speed, double turning_angle);

// Wheel turning rates [rad/s] from per-wheel linear speeds plus a yaw correction.
WheelPair mix_linear_and_angular(
  const WheelGeometry & geometry, const WheelPair & linear, double angular_speed);

// Twist that open-loop odometry integrates for the given wheel turning rates.
// Not an exact inverse of mix_linear_and_angular (see the "divide by 2" note).
Twist2D open_loop_twist(const WheelGeometry & geometry, const WheelPair & turning_rate);

struct AckerCommand
{
  double speed = 0.0;                    // m/s
  double steering_angle = 0.0;           // rad
  double steering_angle_velocity = 0.0;  // rad/s, authority given to the steering loop
};

class AckerDiffCore
{
public:
  struct Config
  {
    WheelGeometry geometry;
    SteeringParams steering;
    PID::Settings pid = PID::getDefault();
    SpeedLimiter linear_limiter;
    SpeedLimiter angle_limiter;
    bool open_loop = false;
    size_t velocity_rolling_window_size = 10;
  };

  struct Measurement
  {
    double steering_angle;      // rad
    WheelPair wheel_position;   // rad, mean per side; ignored in open loop
  };

  enum class Status
  {
    ok,
    invalid_steering_angle,  // nothing beyond command limiting was computed
    invalid_wheel_position,  // limited_command is valid; odometry and wheel rates are not
  };

  struct Output
  {
    Status status = Status::ok;
    WheelPair wheel_rates{0.0, 0.0};  // rad/s, to command when status is ok
    AckerCommand limited_command;     // steering_angle_velocity holds the absolute authority
    double angular_correction = 0.0;  // steering loop output, rad/s
    bool steering_limit_reset = false;  // loop reset: it would steer further past a limit
    bool linear_stopped = false;        // linear speed zeroed: steering error too large
  };

  // Applies the configuration and resets all state.
  void configure(const Config & config);

  // Resets pose, steering loop and command history. Keeps the odometry timestamp.
  void reset();

  // Forget the last limited linear speed, as when the wheels were stopped externally.
  void halt();

  // One control cycle. dt is the time since the previous step, now the current
  // time on the same base, both in seconds.
  Output step(
    const AckerCommand & command, const Measurement & measurement, double dt, double now);

  const Odometry & odometry() const { return odometry_; }
  const Config & config() const { return config_; }

private:
  Config config_;
  PID pid_controller_;
  Odometry odometry_;
  std::array<AckerCommand, 2> previous_commands_{};  // [0] second to last, [1] last
  double last_limited_linear_command_ = 0.0;
};

}  // namespace acker_diff_controller
