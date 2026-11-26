// Tests for re-rooting the front-axle odometry at the rear axle centre
// (docs/adr/0002 in the workspace repository).
//
// Convention: a positive steering angle steers right, so the front axle is
// turned clockwise relative to the chassis and chassis heading = front-axle
// heading + steering angle. Expected values are derived by hand with
// L = 0.67, cos(0.3) = 0.955336, sin(0.3) = 0.295520.

#include <gtest/gtest.h>

#include <cmath>

#include "acker_diff_controller/acker_diff_core.hpp"

using acker_diff_controller::AckerCommand;
using acker_diff_controller::AckerDiffCore;
using acker_diff_controller::front_axle_to_rear_axle_centre;
using acker_diff_controller::front_axle_twist_to_rear_axle_centre;
using acker_diff_controller::PlanarTwist;
using acker_diff_controller::Pose2D;

namespace
{
constexpr double kWheelbase = 0.67;
constexpr double kTol = 1e-6;
constexpr double kDt = 0.02;

AckerDiffCore configured_core()
{
  AckerDiffCore::Config c;
  c.geometry.separation = 0.62;
  c.geometry.radius = 0.125;
  c.geometry.wheelbase = kWheelbase;
  AckerDiffCore core;
  core.configure(c);
  return core;
}

// Wheels stand still, so the front-axle odometry stays at the origin.
AckerDiffCore::Measurement standing_with_steering_at(double steering_angle)
{
  return AckerDiffCore::Measurement{steering_angle, {0.0, 0.0}};
}
}  // namespace

TEST(RearAxleCentrePose, straight_ahead_sits_one_wheelbase_behind)
{
  const Pose2D rear = front_axle_to_rear_axle_centre(Pose2D{1.0, 2.0, 0.0}, 0.0, kWheelbase);
  EXPECT_NEAR(rear.x, 1.0 - 0.67, kTol);
  EXPECT_NEAR(rear.y, 2.0, kTol);
  EXPECT_NEAR(rear.heading, 0.0, kTol);
}

TEST(RearAxleCentrePose, steering_right_turns_chassis_left_of_front_axle)
{
  const Pose2D rear = front_axle_to_rear_axle_centre(Pose2D{0.0, 0.0, 0.0}, 0.3, kWheelbase);
  EXPECT_NEAR(rear.heading, 0.3, kTol);
  EXPECT_NEAR(rear.x, -0.640075, kTol);
  EXPECT_NEAR(rear.y, -0.197998, kTol);
}

TEST(RearAxleCentreTwist, rolling_without_slip_has_no_lateral_velocity)
{
  // Rear wheels don't slip when the chassis yaws at -v*sin(delta)/L.
  const double v = 1.0;
  const double chassis_yaw_rate = -0.295520 / 0.67;  // -0.441075
  const PlanarTwist rear = front_axle_twist_to_rear_axle_centre(
    v, chassis_yaw_rate, 0.3, 0.0, kWheelbase);
  EXPECT_NEAR(rear.linear_x, 0.955336, kTol);
  EXPECT_NEAR(rear.linear_y, 0.0, kTol);
  EXPECT_NEAR(rear.angular, chassis_yaw_rate, kTol);
}

TEST(RearAxleCentreTwist, steering_in_place_swings_the_rear_sideways)
{
  // Front axle stands still; the chassis turns left about the pivot at the
  // steering rate, so the rear axle centre moves right at rate * L.
  const PlanarTwist rear = front_axle_twist_to_rear_axle_centre(0.0, 0.0, 0.0, 0.5, kWheelbase);
  EXPECT_NEAR(rear.linear_x, 0.0, kTol);
  EXPECT_NEAR(rear.linear_y, -0.335, kTol);
  EXPECT_NEAR(rear.angular, 0.5, kTol);
}

// cos(0.2) = 0.980067, sin(0.2) = 0.198669
TEST(RearAxleCentreState, standing_robot_puts_rear_axle_behind_the_steered_front)
{
  AckerDiffCore core = configured_core();
  core.step(AckerCommand{}, standing_with_steering_at(0.2), kDt, 1.0);

  const auto state = core.rear_axle_centre();
  EXPECT_NEAR(state.pose.x, -0.656645, kTol);
  EXPECT_NEAR(state.pose.y, -0.133108, kTol);
  EXPECT_NEAR(state.pose.heading, 0.2, kTol);
  // First cycle: no previous steering angle, so no steering rate yet.
  EXPECT_NEAR(state.twist.linear_y, 0.0, kTol);
  EXPECT_NEAR(state.twist.angular, 0.0, kTol);
}

TEST(RearAxleCentreState, steering_rate_comes_from_consecutive_measurements)
{
  AckerDiffCore core = configured_core();
  core.step(AckerCommand{}, standing_with_steering_at(0.2), kDt, 1.0);
  core.step(AckerCommand{}, standing_with_steering_at(0.21), kDt, 1.0 + kDt);

  const auto state = core.rear_axle_centre();
  EXPECT_NEAR(state.twist.angular, 0.5, kTol);
  EXPECT_NEAR(state.twist.linear_y, -0.335, kTol);
}

TEST(RearAxleCentreState, invalid_steering_angle_keeps_the_last_state)
{
  AckerDiffCore core = configured_core();
  core.step(AckerCommand{}, standing_with_steering_at(0.2), kDt, 1.0);
  core.step(AckerCommand{}, standing_with_steering_at(NAN), kDt, 1.0 + kDt);
  core.step(AckerCommand{}, standing_with_steering_at(0.21), kDt, 1.0 + 2 * kDt);

  // The rate spans both cycles since the last valid angle: 0.01 / 0.04.
  const auto state = core.rear_axle_centre();
  EXPECT_NEAR(state.pose.heading, 0.21, kTol);
  EXPECT_NEAR(state.twist.angular, 0.25, kTol);
}

TEST(RearAxleCentreState, reset_forgets_the_previous_steering_angle)
{
  AckerDiffCore core = configured_core();
  core.step(AckerCommand{}, standing_with_steering_at(0.2), kDt, 1.0);
  core.reset();
  core.step(AckerCommand{}, standing_with_steering_at(0.3), kDt, 1.0 + kDt);

  EXPECT_NEAR(core.rear_axle_centre().twist.angular, 0.0, kTol);
}
