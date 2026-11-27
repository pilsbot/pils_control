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

struct Pose2D
{
  double x;
  double y;
  double heading;  // rad
};

// Pose of the rear axle centre for a given front axle pose (its pivot point
// and heading). A positive steering angle steers right, so the chassis heading
// is the front-axle heading plus the steering angle (docs/adr/0002).
Pose2D front_axle_to_rear_axle_centre(
  const Pose2D & front_axle, double steering_angle, double wheelbase);

// Velocity of a planar rigid body, expressed in its own frame.
struct PlanarTwist
{
  double linear_x;  // m/s, forward
  double linear_y;  // m/s, left
  double angular;   // rad/s, counter-clockwise
};

// Twist of the rear axle centre in the chassis frame, from the front axle's
// forward speed and yaw rate. A rigid-body transform, not a vehicle model: the
// chassis yaws at front yaw rate + steering rate, and the rear axle centre
// moves with the pivot plus that rotation about it.
PlanarTwist front_axle_twist_to_rear_axle_centre(
  double front_linear, double front_angular, double steering_angle,
  double steering_rate, double wheelbase);

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
    // Invalid input shorter than this (s) keeps the ramp and steering loop;
    // a longer fault restarts them from standstill.
    double invalid_input_grace_period = 0.25;
  };

  struct Measurement
  {
    double steering_angle;      // rad
    WheelPair wheel_position;   // rad, mean per side; ignored in open loop
  };

  // On any status but ok the caller must halt the wheels for this cycle
  // (wheel_rates are 0). A fault shorter than invalid_input_grace_period keeps
  // the linear ramp and steering loop, so a one-cycle glitch does not brake
  // hard. Once a fault lasts the grace period, the core restarts both, so
  // nothing jumps when valid measurements return.
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

  struct BaseState
  {
    Pose2D pose{0.0, 0.0, 0.0};
    PlanarTwist twist{0.0, 0.0, 0.0};
  };

  const Odometry & odometry() const { return odometry_; }

  // Odometry re-rooted at the rear axle centre, in the odometry frame. Updated
  // by every step with a valid steering angle; the steering rate is the
  // difference to the previous valid angle (zero on the first one after reset).
  const BaseState & rear_axle_centre() const { return rear_axle_centre_; }
  const Config & config() const { return config_; }

private:
  Config config_;
  PID pid_controller_;
  Odometry odometry_;
  std::array<AckerCommand, 2> previous_commands_{};  // [0] second to last, [1] last
  double last_limited_linear_command_ = 0.0;

  void update_rear_axle_centre(double steering_angle, double now);
  void handle_invalid_input(double dt);
  double invalid_input_duration_ = 0.0;  // s, accumulated dt of the current fault
  BaseState rear_axle_centre_;
  bool has_last_steering_angle_ = false;
  double last_steering_angle_ = 0.0;
  double last_steering_time_ = 0.0;
};

}  // namespace acker_diff_controller
