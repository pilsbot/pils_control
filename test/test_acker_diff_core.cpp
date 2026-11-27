// Tests for the Acker-diff steering core, through its public interface only.
//
// Where a value can be derived by hand, the test asserts the derived value.
// The multi-step steering scenario records what the core computes (generated
// with GENERATE_GOLDEN=1). The original controller took dt from a steady
// clock, so its full loop could not be recorded deterministically before the
// extraction; these goldens pin the extracted behaviour from here on.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "acker_diff_controller/acker_diff_core.hpp"

using acker_diff_controller::AckerCommand;
using acker_diff_controller::AckerDiffCore;
using acker_diff_controller::SpeedLimiter;
using acker_diff_controller::WheelPair;

namespace
{
constexpr double kDt = 0.02;  // 50 Hz, as configured on the robot

bool generate() { return std::getenv("GENERATE_GOLDEN") != nullptr; }

// Mirrors pilsbot_hw_ros/pilsbot_control/config/acker_diff_controller.yaml
AckerDiffCore::Config pilsbot_config()
{
  AckerDiffCore::Config c;
  c.geometry.separation = 0.62;
  c.geometry.radius = 0.125;
  c.geometry.wheelbase = 0.67;

  c.steering.has_position_limit = true;
  c.steering.max_angle = 1.45;
  c.steering.min_angle = -1.45;
  c.steering.max_angular_velocity = 1.0;
  c.steering.max_angular_acceleration = 5.0;
  c.steering.limit_linear_at_angle_diff_to_setpoint = 1.2;

  c.pid = PID::Settings{2.0, 0.05, 0.005, 1.0, -1.0, 5.0, 0.3};

  c.linear_limiter = SpeedLimiter(true, true, false, NAN, 1.5, NAN, 0.3);
  // Angle limiter as the controller builds it: angular velocity goes into the
  // acceleration slot, so it limits the steering rate.
  c.angle_limiter = SpeedLimiter(true, true, false, -1.45, 1.45, NAN, 1.0);
  return c;
}

AckerDiffCore::Measurement straight_at(double left_pos, double right_pos, double angle = 0.0)
{
  return AckerDiffCore::Measurement{angle, WheelPair{left_pos, right_pos}};
}
}  // namespace

TEST(AckerDiffCore, linear_speed_ramps_at_configured_acceleration)
{
  AckerDiffCore core;
  core.configure(pilsbot_config());

  double t = 100.0;
  for (int k = 1; k <= 5; ++k) {
    t += kDt;
    const auto out = core.step(AckerCommand{1.0, 0.0, 0.0}, straight_at(0, 0), kDt, t);
    ASSERT_EQ(out.status, AckerDiffCore::Status::ok);
    const double expected_speed = 0.3 * kDt * k;  // max_acceleration * dt per step
    EXPECT_NEAR(out.limited_command.speed, expected_speed, 1e-12);
    EXPECT_NEAR(out.wheel_rates.left, expected_speed / 0.125, 1e-12);
    EXPECT_NEAR(out.wheel_rates.right, expected_speed / 0.125, 1e-12);
    EXPECT_EQ(out.angular_correction, 0.0);
  }
}

TEST(AckerDiffCore, steering_loop_is_passive_without_authority)
{
  AckerDiffCore core;
  core.configure(pilsbot_config());

  // Large angle error, but steering_angle_velocity 0 gives the loop no authority.
  const auto out = core.step(AckerCommand{0.0, 0.0, 0.0}, straight_at(0, 0, 0.8), kDt, 100.02);
  EXPECT_EQ(out.status, AckerDiffCore::Status::ok);
  EXPECT_EQ(out.angular_correction, 0.0);
  EXPECT_EQ(out.wheel_rates.left, 0.0);
  EXPECT_EQ(out.wheel_rates.right, 0.0);
}

TEST(AckerDiffCore, correction_is_capped_by_requested_steering_rate)
{
  auto config = pilsbot_config();
  config.pid.max_dv = NAN;  // otherwise the output rate limit caps the first step first
  AckerDiffCore core;
  core.configure(config);

  // Error of 1 rad with Kp=2 would ask for more than 2 rad/s; the request allows 0.4.
  const auto out = core.step(AckerCommand{0.0, 0.0, 0.4}, straight_at(0, 0, 1.0), kDt, 100.02);
  EXPECT_NEAR(std::abs(out.angular_correction), 0.4, 1e-12);
  EXPECT_EQ(out.limited_command.steering_angle_velocity, 0.4);
}

TEST(AckerDiffCore, resets_loop_instead_of_steering_past_a_limit)
{
  // The reset only triggers when the setpoint lies beyond the limit the axle is
  // already at. With the robot's config the angle limiter clamps setpoints to
  // max_angle, so use a narrower steering limit and an unlimited setpoint here.
  auto config = pilsbot_config();
  config.steering.max_angle = 0.3;
  config.steering.min_angle = -0.3;
  config.angle_limiter = SpeedLimiter();
  AckerDiffCore core;
  core.configure(config);

  const auto out = core.step(AckerCommand{0.0, 0.5, 1.0}, straight_at(0, 0, 0.35), kDt, 100.02);
  EXPECT_TRUE(out.steering_limit_reset);
  EXPECT_EQ(out.angular_correction, 0.0);
}

TEST(AckerDiffCore, stops_linear_motion_when_correction_exceeds_gate)
{
  auto config = pilsbot_config();
  // The output rate limit (max_dv 5 rad/s^2) allows 0.1 rad/s in the first step.
  config.steering.limit_linear_at_angle_diff_to_setpoint = 0.05;
  AckerDiffCore core;
  core.configure(config);

  const auto out = core.step(AckerCommand{1.0, 0.0, 1.0}, straight_at(0, 0, 0.5), kDt, 100.02);
  EXPECT_TRUE(out.linear_stopped);
  EXPECT_EQ(out.limited_command.speed, 0.0);
}

TEST(AckerDiffCore, robot_gate_cannot_trigger_with_current_robot_config)
{
  // Characterisation: the correction is capped at max_angular_velocity (1.0),
  // which is below the configured gate (1.2), so linear motion is never stopped.
  AckerDiffCore core;
  core.configure(pilsbot_config());
  const auto out = core.step(AckerCommand{1.0, -1.45, 5.0}, straight_at(0, 0, 1.4), kDt, 100.02);
  EXPECT_FALSE(out.linear_stopped);
}

TEST(AckerDiffCore, invalid_steering_angle_stops_the_cycle)
{
  AckerDiffCore core;
  core.configure(pilsbot_config());

  const auto out = core.step(AckerCommand{1.0, 0.3, 1.0}, straight_at(0, 0, NAN), kDt, 100.02);
  EXPECT_EQ(out.status, AckerDiffCore::Status::invalid_steering_angle);
  EXPECT_EQ(core.odometry().getX(), 0.0);
}

TEST(AckerDiffCore, invalid_wheel_position_keeps_odometry)
{
  AckerDiffCore core;
  core.configure(pilsbot_config());

  const auto out = core.step(AckerCommand{1.0, 0.0, 0.0}, straight_at(NAN, 0.0), kDt, 100.02);
  EXPECT_EQ(out.status, AckerDiffCore::Status::invalid_wheel_position);
  EXPECT_NEAR(out.limited_command.speed, 0.006, 1e-12);  // limiting still happened
  EXPECT_EQ(core.odometry().getX(), 0.0);
}

// pils_control#3: the adapter halts the wheels on invalid input, so the core
// must not resume from the speed it had before. The ramp restarts from zero.
TEST(AckerDiffCore, linear_ramp_restarts_after_invalid_input)
{
  for (const auto invalid : {straight_at(0, 0, NAN), straight_at(NAN, 0)}) {
    AckerDiffCore core;
    core.configure(pilsbot_config());
    double t = 100.0;
    for (int k = 0; k < 50; ++k) {
      t += kDt;
      core.step(AckerCommand{1.0, 0.0, 0.0}, straight_at(0, 0), kDt, t);
    }
    t += kDt;
    EXPECT_NE(core.step(AckerCommand{1.0, 0.0, 0.0}, invalid, kDt, t).status,
      AckerDiffCore::Status::ok);
    t += kDt;
    const auto out = core.step(AckerCommand{1.0, 0.0, 0.0}, straight_at(0, 0), kDt, t);
    EXPECT_NEAR(out.limited_command.speed, 0.3 * kDt, 1e-12);
  }
}

// pils_control#3: the steering loop restarts too, so the correction does not
// jump to its pre-fault value when valid data returns.
TEST(AckerDiffCore, steering_loop_restarts_after_invalid_input)
{
  for (const auto invalid : {straight_at(0, 0, NAN), straight_at(NAN, 0, 0.5)}) {
    AckerDiffCore core;
    core.configure(pilsbot_config());
    double t = 100.0;
    double correction = 0.0;
    for (int k = 0; k < 50; ++k) {
      t += kDt;
      correction =
        core.step(AckerCommand{0.0, 0.0, 1.0}, straight_at(0, 0, 0.5), kDt, t).angular_correction;
    }
    ASSERT_NEAR(std::abs(correction), 1.0, 1e-9) << "loop should be saturated";
    t += kDt;
    core.step(AckerCommand{0.0, 0.0, 1.0}, invalid, kDt, t);
    t += kDt;
    const auto out = core.step(AckerCommand{0.0, 0.0, 1.0}, straight_at(0, 0, 0.5), kDt, t);
    // A fresh loop is limited by max_dv (5 rad/s^2) on its first step.
    EXPECT_LE(std::abs(out.angular_correction), 5.0 * kDt + 1e-12);
  }
}

// pils_control#4: a hard stop (cmd_vel timeout) bypasses the linear limits.
TEST(AckerDiffCore, hard_stop_bypasses_the_acceleration_limit)
{
  AckerDiffCore core;
  core.configure(pilsbot_config());
  double t = 100.0;
  for (int k = 0; k < 100; ++k) {
    t += kDt;
    core.step(AckerCommand{1.0, 0.3, 1.0}, straight_at(0, 0, 0.3), kDt, t);
  }

  AckerCommand stop{0.0, 0.3, 0.0};
  stop.hard_stop = true;
  t += kDt;
  const auto out = core.step(stop, straight_at(0, 0, 0.3), kDt, t);
  EXPECT_EQ(out.status, AckerDiffCore::Status::ok);
  EXPECT_EQ(out.limited_command.speed, 0.0);
  EXPECT_EQ(out.wheel_rates.left, 0.0);
  EXPECT_EQ(out.wheel_rates.right, 0.0);

  // The next normal command ramps up from standstill.
  t += kDt;
  const auto resumed = core.step(AckerCommand{1.0, 0.3, 1.0}, straight_at(0, 0, 0.3), kDt, t);
  EXPECT_NEAR(resumed.limited_command.speed, 0.3 * kDt, 1e-12);
}

TEST(AckerDiffCore, open_loop_odometry_ignores_wheel_positions)
{
  auto config = pilsbot_config();
  config.open_loop = true;
  AckerDiffCore core;
  core.configure(config);

  core.step(AckerCommand{1.0, 0.0, 0.0}, straight_at(NAN, NAN), kDt, 100.00);
  const auto out = core.step(AckerCommand{1.0, 0.0, 0.0}, straight_at(NAN, NAN), kDt, 100.02);
  EXPECT_EQ(out.status, AckerDiffCore::Status::ok);
  // Characterisation: odometry starts at timestamp 0, so the first step integrates
  // 0.006 m/s over 100 s. The second step adds 0.012 m/s over 0.02 s.
  EXPECT_NEAR(core.odometry().getX(), 0.006 * 100.0 + 0.012 * 0.02, 1e-9);
}

TEST(AckerDiffCore, reset_clears_pose_and_command_history)
{
  AckerDiffCore core;
  core.configure(pilsbot_config());
  double t = 100.0;
  for (int k = 0; k < 5; ++k) {
    t += kDt;
    core.step(AckerCommand{1.0, 0.0, 0.0}, straight_at(0.01 * k, 0.01 * k), kDt, t);
  }
  core.reset();
  EXPECT_EQ(core.odometry().getX(), 0.0);
  // Ramp restarts from zero after a reset.
  const auto out = core.step(AckerCommand{1.0, 0.0, 0.0}, straight_at(0.05, 0.05), kDt, t + kDt);
  EXPECT_NEAR(out.limited_command.speed, 0.006, 1e-12);
}

// Scripted steering manoeuvre: drive at 0.5 m/s, steer to 0.4 rad, back to 0.
TEST(AckerDiffCore, steering_manoeuvre_golden)
{
  AckerDiffCore core;
  core.configure(pilsbot_config());

  struct Step { double cmd_angle; double measured_angle; double left_pos; double right_pos; };
  const std::vector<Step> steps = {
    {0.0, 0.00, 0.00, 0.00},
    {0.4, 0.00, 0.01, 0.01},
    {0.4, 0.02, 0.03, 0.03},
    {0.4, 0.06, 0.06, 0.07},
    {0.4, 0.12, 0.10, 0.12},
    {0.4, 0.20, 0.14, 0.18},
    {0.4, 0.28, 0.18, 0.25},
    {0.4, 0.35, 0.22, 0.32},
    {0.4, 0.39, 0.26, 0.39},
    {0.4, 0.41, 0.30, 0.46},
    {0.0, 0.40, 0.34, 0.53},
    {0.0, 0.33, 0.39, 0.59},
    {0.0, 0.22, 0.44, 0.64},
    {0.0, 0.10, 0.49, 0.68},
    {0.0, 0.02, 0.54, 0.72},
  };
  struct Golden { double left; double right; double correction; double x; double y; double heading; };
  const std::vector<Golden> golden = {
    {0.048000000000000001, 0.048000000000000001, -0, 0, 0, 0},
    {0.20764959999999999, -0.015649599999999986, -0.045019999999999998, 0.00125, 0, 0},
    {0.52958912362605537, -0.24159963043718963, -0.14502000000000001, 0.0037499999999999999, 0, 0},
    {0.83422920266689338, -0.45025410642878327, -0.24502000000000002, 0.008124997036101235, 4.41028076409444e-06, 0.0020161290322580662},
    {1.138863772484112, -0.65891240945052687, -0.34501999999999999, 0.013749970361030942, 2.1421340662962673e-05, 0.0040322580645161272},
    {1.2827874572787425, -0.70687149521315151, -0.38022, 0.019999851805483092, 5.9223503921073169e-05, 0.0080645161290322544},
    {1.0914068822556353, -0.41954031873007691, -0.28022000000000002, 0.026874418657741334, 0.00013545670446105253, 0.014112903225806453},
    {0.90001667382341655, -0.13221583417850907, -0.18022000000000002, 0.033748398679194686, 0.00025326829851141466, 0.020161290322580648},
    {0.70861545979400986, 0.15510100523915868, -0.080220000000000014, 0.040621540399891624, 0.0004126539761853265, 0.026209677419354843},
    {0.51720186958164527, 0.44240924743358889, 0.019779999999999992, 0.047493592380548094, 0.00061360790669653183, 0.032258064516129045},
    {0.51341144176896969, 0.54211899418283627, 0.042310000000000049, 0.05436430322174586, 0.00085612273857249609, 0.038306451612903233},
    {0.22307119999999986, 0.9289288, 0.14231000000000005, 0.061233989635387412, 0.0011263403691619427, 0.040322580645161289},
    {0.023071199999999847, 1.2249288000000003, 0.24231000000000005, 0.067478909353440308, 0.0013782882111337273, 0.040322580645161303},
    {0.24980480000000005, 1.0941952000000001, 0.17024, 0.073099561873692487, 0.0015993753634341839, 0.038306451612903247},
    {0.54580480000000009, 0.89419520000000008, 0.070239999999999997, 0.078720648710525476, 0.0018091301133505966, 0.036290322580645157},
  };

  double t = 100.0;
  for (size_t i = 0; i < steps.size(); ++i) {
    t += kDt;
    const auto & s = steps[i];
    const auto out = core.step(
      AckerCommand{0.5, s.cmd_angle, 1.0},
      AckerDiffCore::Measurement{s.measured_angle, WheelPair{s.left_pos, s.right_pos}}, kDt, t);
    ASSERT_EQ(out.status, AckerDiffCore::Status::ok);
    const auto & o = core.odometry();
    if (generate() || golden.empty()) {
      std::printf("    {%.17g, %.17g, %.17g, %.17g, %.17g, %.17g},\n",
        out.wheel_rates.left, out.wheel_rates.right, out.angular_correction,
        o.getX(), o.getY(), o.getHeading());
      continue;
    }
    EXPECT_DOUBLE_EQ(out.wheel_rates.left, golden[i].left) << "step " << i;
    EXPECT_DOUBLE_EQ(out.wheel_rates.right, golden[i].right) << "step " << i;
    EXPECT_DOUBLE_EQ(out.angular_correction, golden[i].correction) << "step " << i;
    EXPECT_DOUBLE_EQ(o.getX(), golden[i].x) << "step " << i;
    EXPECT_DOUBLE_EQ(o.getY(), golden[i].y) << "step " << i;
    EXPECT_DOUBLE_EQ(o.getHeading(), golden[i].heading) << "step " << i;
  }
  if (!generate()) {
    ASSERT_FALSE(golden.empty()) << "golden values missing; run with GENERATE_GOLDEN=1";
  }
}
