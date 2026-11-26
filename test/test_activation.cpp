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
