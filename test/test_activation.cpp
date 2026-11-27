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

// Gives the tests a look at the command the subscriber stored for update().
class InspectableController : public acker_diff_controller::AckerDiffController
{
public:
  std::shared_ptr<ackermann_msgs::msg::AckermannDriveStamped> stored_command()
  {
    std::shared_ptr<ackermann_msgs::msg::AckermannDriveStamped> msg;
    received_command_msg_ptr_.get(msg);
    return msg;
  }
};

class Activation : public ::testing::Test
{
protected:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }
  static void TearDownTestCase() { rclcpp::shutdown(); }

  void SetUp() override
  {
    controller_ = std::make_unique<InspectableController>();
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

  // Receives one /odom message while updating the controller.
  nav_msgs::msg::Odometry::SharedPtr receive_odometry()
  {
    auto listener = std::make_shared<rclcpp::Node>("odometry_listener");
    nav_msgs::msg::Odometry::SharedPtr odom;
    auto odom_sub = listener->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, [&odom](nav_msgs::msg::Odometry::SharedPtr msg) { odom = msg; });
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!odom && std::chrono::steady_clock::now() < deadline) {
      update_once();
      rclcpp::spin_some(listener);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return odom;
  }

  std::unique_ptr<InspectableController> controller_;
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

// pils_control#6: the unstamped callback edited the stored message in place
// while update() read it. It must store a new message instead.
TEST_F(Activation, unstamped_command_is_stored_as_a_new_message)
{
  auto node = controller_->get_node();
  node->set_parameter(rclcpp::Parameter("use_stamped_vel", false));
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  assign_interfaces();
  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);
  const auto before = controller_->stored_command();
  ASSERT_TRUE(before);

  auto commander = std::make_shared<rclcpp::Node>("commander");
  auto publisher = commander->create_publisher<ackermann_msgs::msg::AckermannDrive>(
    "/pilsbot_velocity_controller/cmd_vel_unstamped", rclcpp::SystemDefaultsQoS());
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (controller_->stored_command()->drive.speed != 0.5 &&
    std::chrono::steady_clock::now() < deadline)
  {
    ackermann_msgs::msg::AckermannDrive command;
    command.speed = 0.5;
    publisher->publish(command);
    rclcpp::spin_some(node->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  const auto after = controller_->stored_command();
  ASSERT_EQ(after->drive.speed, 0.5) << "command never arrived";
  EXPECT_NE(after, before) << "the stored message was edited in place";
  EXPECT_EQ(before->drive.speed, 0.0) << "the stored message was edited in place";

  ASSERT_EQ(update_once(), controller_interface::return_type::OK);
  EXPECT_GT(left_velocity_, 0.0);
}

// pils_control#6: a covariance diagonal must have 6 entries; anything else
// used to be copied unchecked into a 6-element array.
TEST_F(Activation, covariance_diagonal_of_wrong_size_fails_configure)
{
  for (const auto & name : {"pose_covariance_diagonal", "twist_covariance_diagonal"}) {
    SetUp();
    auto node = controller_->get_node();
    node->set_parameter(rclcpp::Parameter(name, std::vector<double>{1, 2, 3, 4, 5}));
    EXPECT_NE(node->configure().id(), State::PRIMARY_STATE_INACTIVE) << name;
  }
}

TEST_F(Activation, covariance_defaults_to_zero_and_takes_configured_diagonal)
{
  auto node = controller_->get_node();
  node->set_parameter(rclcpp::Parameter(
    "pose_covariance_diagonal", std::vector<double>{1, 2, 3, 4, 5, 6}));
  ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  assign_interfaces();
  ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);

  const auto odom = receive_odometry();
  ASSERT_TRUE(odom) << "no /odom received";
  for (size_t i = 0; i < 36; ++i) {
    const bool diagonal = i % 7 == 0;
    EXPECT_EQ(odom->pose.covariance[i], diagonal ? 1.0 + i / 7 : 0.0) << "pose " << i;
    EXPECT_EQ(odom->twist.covariance[i], 0.0) << "twist " << i;
  }
}
