// Lifecycle test for the ROS adapter: the controller must find its interfaces,
// activate, update, and survive a deactivate/activate cycle. This guards the
// Humble-specific behaviour that the core tests cannot see: interface lookup by
// joint name (Humble's get_name() returns "joint/interface") and interfaces
// being re-loaned on every activation.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "acker_diff_controller/acker_diff_controller.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "rclcpp/rclcpp.hpp"

using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

namespace
{
const std::string kLeft = "front_wheel_left_joint";
const std::string kRight = "front_wheel_right_joint";
const std::string kSteering = "steering_axle_joint";

class Activation : public ::testing::Test
{
protected:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }
  static void TearDownTestCase() { rclcpp::shutdown(); }

  void SetUp() override
  {
    controller_ = std::make_unique<acker_diff_controller::AckerDiffController>();
    ASSERT_EQ(controller_->init("pilsbot_velocity_controller"),
      controller_interface::return_type::OK);
    auto node = controller_->get_node();
    node->set_parameter(rclcpp::Parameter("left_wheel_names", std::vector<std::string>{kLeft}));
    node->set_parameter(rclcpp::Parameter("right_wheel_names", std::vector<std::string>{kRight}));
    node->set_parameter(rclcpp::Parameter("steering_axle_name", kSteering));
    node->set_parameter(rclcpp::Parameter("wheel_separation", 0.62));
    node->set_parameter(rclcpp::Parameter("wheel_base_distance", 0.67));
    node->set_parameter(rclcpp::Parameter("wheel_radius", 0.125));
    node->set_parameter(rclcpp::Parameter("wheels_per_side", 1));
  }

  void assign_interfaces()
  {
    std::vector<hardware_interface::LoanedCommandInterface> commands;
    commands.emplace_back(left_velocity_command_);
    commands.emplace_back(right_velocity_command_);
    std::vector<hardware_interface::LoanedStateInterface> states;
    states.emplace_back(left_position_state_);
    states.emplace_back(right_position_state_);
    states.emplace_back(steering_position_state_);
    controller_->assign_interfaces(std::move(commands), std::move(states));
  }

  controller_interface::return_type update_once()
  {
    return controller_->update(
      controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.02));
  }

  // Configures and activates the controller, then drives it with a stamped
  // command until the wheels turn. Returns false if they never do.
  bool drive_forward(double speed)
  {
    if (controller_->get_node()->configure().id() != State::PRIMARY_STATE_INACTIVE) {
      return false;
    }
    assign_interfaces();
    if (controller_->get_node()->activate().id() != State::PRIMARY_STATE_ACTIVE) {
      return false;
    }
    commander_ = std::make_shared<rclcpp::Node>("commander");
    command_publisher_ = commander_->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      "/pilsbot_velocity_controller/cmd_vel", rclcpp::SystemDefaultsQoS());
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
      send_and_update(speed);
      if (left_velocity_ > 0.0 && right_velocity_ > 0.0) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  // One control cycle with a fresh stamped command (drive_forward sets up the publisher).
  void send_and_update(double speed)
  {
    ackermann_msgs::msg::AckermannDriveStamped command;
    command.header.stamp = controller_->get_node()->now();
    command.drive.speed = speed;
    command_publisher_->publish(command);
    rclcpp::spin_some(controller_->get_node()->get_node_base_interface());
    update_once();
  }

  rclcpp::Node::SharedPtr commander_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr command_publisher_;

  double left_position_ = 0.0;
  double right_position_ = 0.0;
  double steering_position_ = 0.0;
  double left_velocity_ = NAN;
  double right_velocity_ = NAN;

  hardware_interface::StateInterface left_position_state_{kLeft, HW_IF_POSITION, &left_position_};
  hardware_interface::StateInterface right_position_state_{kRight, HW_IF_POSITION, &right_position_};
  hardware_interface::StateInterface steering_position_state_{
    kSteering, HW_IF_POSITION, &steering_position_};
  hardware_interface::CommandInterface left_velocity_command_{kLeft, HW_IF_VELOCITY, &left_velocity_};
  hardware_interface::CommandInterface right_velocity_command_{
    kRight, HW_IF_VELOCITY, &right_velocity_};

  std::unique_ptr<acker_diff_controller::AckerDiffController> controller_;
};
}  // namespace

TEST_F(Activation, finds_interfaces_by_joint_name_and_updates)
{
  ASSERT_EQ(controller_->get_node()->configure().id(), State::PRIMARY_STATE_INACTIVE);
  assign_interfaces();
  ASSERT_EQ(controller_->get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE);

  ASSERT_EQ(update_once(), controller_interface::return_type::OK);
  // No command received yet: the wheels are commanded to stand still.
  EXPECT_EQ(left_velocity_, 0.0);
  EXPECT_EQ(right_velocity_, 0.0);
}

TEST_F(Activation, survives_deactivate_activate_cycle)
{
  ASSERT_EQ(controller_->get_node()->configure().id(), State::PRIMARY_STATE_INACTIVE);
  assign_interfaces();
  ASSERT_EQ(controller_->get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(update_once(), controller_interface::return_type::OK);

  left_velocity_ = right_velocity_ = 1.0;
  ASSERT_EQ(controller_->get_node()->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(left_velocity_, 0.0) << "deactivation must halt the wheels";
  EXPECT_EQ(right_velocity_, 0.0) << "deactivation must halt the wheels";
  controller_->release_interfaces();

  assign_interfaces();
  ASSERT_EQ(controller_->get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(update_once(), controller_interface::return_type::OK);
}

// docs/adr/0002: odometry is published for the rear axle centre, as
// odom -> base_footprint. Wheels stand still and the steering is at 0.2 rad,
// so the rear axle centre sits one wheelbase behind the pivot along the
// chassis heading: (-0.67 cos 0.2, -0.67 sin 0.2), yaw 0.2.
TEST_F(Activation, publishes_rear_axle_centre_odometry_as_base_footprint)
{
  auto listener = std::make_shared<rclcpp::Node>("odometry_listener");
  nav_msgs::msg::Odometry::SharedPtr odom;
  tf2_msgs::msg::TFMessage::SharedPtr tf;
  auto odom_sub = listener->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, [&odom](nav_msgs::msg::Odometry::SharedPtr msg) { odom = msg; });
  auto tf_sub = listener->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", 10, [&tf](tf2_msgs::msg::TFMessage::SharedPtr msg) { tf = msg; });

  ASSERT_EQ(controller_->get_node()->configure().id(), State::PRIMARY_STATE_INACTIVE);
  assign_interfaces();
  ASSERT_EQ(controller_->get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE);
  steering_position_ = 0.2;

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while ((!odom || !tf) && std::chrono::steady_clock::now() < deadline) {
    ASSERT_EQ(update_once(), controller_interface::return_type::OK);
    rclcpp::spin_some(listener);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_TRUE(odom) << "no /odom received";
  ASSERT_TRUE(tf) << "no /tf received";

  constexpr double kTol = 1e-6;
  EXPECT_EQ(odom->header.frame_id, "odom");
  EXPECT_EQ(odom->child_frame_id, "base_footprint");
  EXPECT_NEAR(odom->pose.pose.position.x, -0.656645, kTol);
  EXPECT_NEAR(odom->pose.pose.position.y, -0.133108, kTol);
  EXPECT_NEAR(odom->pose.pose.orientation.z, 0.0998334, kTol);  // sin(0.1)
  EXPECT_NEAR(odom->pose.pose.orientation.w, 0.9950042, kTol);  // cos(0.1)

  ASSERT_EQ(tf->transforms.size(), 1u);
  const auto & transform = tf->transforms.front();
  EXPECT_EQ(transform.header.frame_id, "odom");
  EXPECT_EQ(transform.child_frame_id, "base_footprint");
  EXPECT_NEAR(transform.transform.translation.x, -0.656645, kTol);
  EXPECT_NEAR(transform.transform.translation.y, -0.133108, kTol);
  EXPECT_NEAR(transform.transform.rotation.z, 0.0998334, kTol);
}

// pils_control#3: the driver writes NaN steering on hoverboard timeouts. The
// controller must not leave the last wheel rates in the command interfaces.
TEST_F(Activation, invalid_steering_angle_halts_the_wheels)
{
  ASSERT_TRUE(drive_forward(0.5)) << "the wheels never turned";

  steering_position_ = NAN;
  EXPECT_EQ(update_once(), controller_interface::return_type::ERROR);
  EXPECT_EQ(left_velocity_, 0.0);
  EXPECT_EQ(right_velocity_, 0.0);
}

TEST_F(Activation, invalid_wheel_position_halts_the_wheels)
{
  ASSERT_TRUE(drive_forward(0.5)) << "the wheels never turned";

  left_position_ = NAN;
  EXPECT_EQ(update_once(), controller_interface::return_type::ERROR);
  EXPECT_EQ(left_velocity_, 0.0);
  EXPECT_EQ(right_velocity_, 0.0);
}

// pils_control#4: on a cmd_vel timeout (e.g. gamepad link loss) the robot must
// stop at once, not ramp down at linear.x.max_acceleration.
TEST_F(Activation, cmd_vel_timeout_stops_the_wheels_at_once)
{
  auto node = controller_->get_node();
  node->set_parameter(rclcpp::Parameter("cmd_vel_timeout", 0.1));
  node->set_parameter(rclcpp::Parameter("linear.x.has_acceleration_limits", true));
  node->set_parameter(rclcpp::Parameter("linear.x.max_acceleration", 0.3));
  ASSERT_TRUE(drive_forward(1.0)) << "the wheels never turned";
  for (int k = 0; k < 25; ++k) {
    send_and_update(1.0);
  }
  ASSERT_GT(left_velocity_, 0.5) << "should have ramped up to a noticeable speed";

  std::this_thread::sleep_for(std::chrono::milliseconds(150));  // past the timeout
  EXPECT_EQ(update_once(), controller_interface::return_type::OK);
  EXPECT_EQ(left_velocity_, 0.0);
  EXPECT_EQ(right_velocity_, 0.0);

  // A fresh command after the timeout ramps up from standstill again.
  send_and_update(1.0);
  EXPECT_GT(left_velocity_, 0.0);
  EXPECT_LE(left_velocity_, 0.3 * 0.02 / 0.125 + 1e-9);
}

// pils_control#4: once a command has timed out it stays timed out, even if ROS
// time steps backwards (e.g. a restarted bag or simulation) and its stamp looks
// fresh again. Only a new message lifts the timeout.
TEST_F(Activation, timed_out_command_stays_timed_out_when_time_steps_back)
{
  auto node = controller_->get_node();
  node->set_parameter(rclcpp::Parameter("use_sim_time", true));
  auto clock_node = std::make_shared<rclcpp::Node>("clock_source");
  auto clock_publisher = clock_node->create_publisher<rosgraph_msgs::msg::Clock>(
    "/clock", rclcpp::ClockQoS());
  const auto set_time = [&](double seconds) {
    rosgraph_msgs::msg::Clock clock;
    clock.clock = rclcpp::Time(static_cast<int64_t>(seconds * 1e9), RCL_ROS_TIME);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (node->now() != rclcpp::Time(clock.clock, RCL_ROS_TIME) &&
      std::chrono::steady_clock::now() < deadline)
    {
      clock_publisher->publish(clock);
      rclcpp::spin_some(node->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return node->now() == rclcpp::Time(clock.clock, RCL_ROS_TIME);
  };
  ASSERT_TRUE(set_time(1000.0)) << "sim time never arrived";
  ASSERT_TRUE(drive_forward(0.5)) << "the wheels never turned";

  ASSERT_TRUE(set_time(1001.0));  // past the 0.5 s timeout
  EXPECT_EQ(update_once(), controller_interface::return_type::OK);
  EXPECT_EQ(left_velocity_, 0.0);

  ASSERT_TRUE(set_time(1000.1));  // time steps back: the stamp looks fresh again
  EXPECT_EQ(update_once(), controller_interface::return_type::OK);
  EXPECT_EQ(left_velocity_, 0.0) << "the timed-out command came back";
  EXPECT_EQ(right_velocity_, 0.0) << "the timed-out command came back";

  // A new message lifts the timeout.
  send_and_update(0.5);
  EXPECT_GT(left_velocity_, 0.0);
}

// The timeout latch must not swallow unstamped commands: on this branch the
// unstamped callback still writes into the stored message (pils_control#6).
TEST_F(Activation, unstamped_command_lifts_the_timeout)
{
  auto node = controller_->get_node();
  node->set_parameter(rclcpp::Parameter("use_stamped_vel", false));
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  assign_interfaces();
  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(update_once(), controller_interface::return_type::OK);  // empty command: timed out
  EXPECT_EQ(left_velocity_, 0.0);

  auto commander = std::make_shared<rclcpp::Node>("commander");
  auto publisher = commander->create_publisher<ackermann_msgs::msg::AckermannDrive>(
    "/pilsbot_velocity_controller/cmd_vel_unstamped", rclcpp::SystemDefaultsQoS());
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (left_velocity_ <= 0.0 && std::chrono::steady_clock::now() < deadline) {
    ackermann_msgs::msg::AckermannDrive command;
    command.speed = 0.5;
    publisher->publish(command);
    rclcpp::spin_some(node->get_node_base_interface());
    update_once();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_GT(left_velocity_, 0.0) << "unstamped command never lifted the timeout";
}

// pils_control#3 (grace period): a one-cycle glitch halts the wheels for that
// cycle only. The adapter must not reset the ramp, so the next valid cycle
// continues near the previous speed instead of ramping up from standstill.
TEST_F(Activation, short_invalid_input_glitch_keeps_the_speed)
{
  for (const bool steering_glitch : {true, false}) {
    SetUp();
    auto node = controller_->get_node();
    node->set_parameter(rclcpp::Parameter("linear.x.has_acceleration_limits", true));
    node->set_parameter(rclcpp::Parameter("linear.x.max_acceleration", 0.3));
    ASSERT_TRUE(drive_forward(1.0)) << "the wheels never turned";
    for (int k = 0; k < 25; ++k) {
      send_and_update(1.0);
    }
    const double before = left_velocity_;
    ASSERT_GT(before, 0.5) << "should have ramped up to a noticeable speed";

    (steering_glitch ? steering_position_ : left_position_) = NAN;
    EXPECT_EQ(update_once(), controller_interface::return_type::ERROR);
    EXPECT_EQ(left_velocity_, 0.0);
    EXPECT_EQ(right_velocity_, 0.0);

    steering_position_ = 0.0;
    left_position_ = 0.0;
    send_and_update(1.0);
    EXPECT_GE(left_velocity_, before) << (steering_glitch ? "steering" : "wheel") << " glitch";
    controller_->get_node()->deactivate();
    controller_->release_interfaces();
  }
}

TEST_F(Activation, invalid_grace_period_fails_configure)
{
  for (const double grace : {-0.1, static_cast<double>(NAN)}) {
    SetUp();
    auto node = controller_->get_node();
    node->set_parameter(rclcpp::Parameter("invalid_input_grace_period", grace));
    EXPECT_NE(node->configure().id(), State::PRIMARY_STATE_INACTIVE) << grace;
  }
}

// On invalid wheel position the wheels are halted; ~/cmd_vel_out must say so
// rather than report the (non-zero) limited speed.
TEST_F(Activation, limited_command_reports_the_halt_on_invalid_wheel_position)
{
  auto node = controller_->get_node();
  node->set_parameter(rclcpp::Parameter("publish_limited_velocity", true));
  ASSERT_TRUE(drive_forward(0.5)) << "the wheels never turned";

  auto listener = std::make_shared<rclcpp::Node>("limited_listener");
  ackermann_msgs::msg::AckermannDriveStamped::SharedPtr limited;
  const rclcpp::Time halted_at = node->now();
  auto sub = listener->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    "/pilsbot_velocity_controller/cmd_vel_out", 10,
    [&limited, halted_at](ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
      if (rclcpp::Time(msg->header.stamp) >= halted_at) {
        limited = msg;
      }
    });

  left_position_ = NAN;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!limited && std::chrono::steady_clock::now() < deadline) {
    send_and_update(0.5);
    EXPECT_EQ(left_velocity_, 0.0);
    rclcpp::spin_some(listener);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_TRUE(limited) << "no ~/cmd_vel_out received";
  EXPECT_EQ(limited->drive.speed, 0.0);
  EXPECT_EQ(limited->drive.steering_angle_velocity, 0.0);
}
