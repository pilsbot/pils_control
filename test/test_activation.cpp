// Lifecycle test for the ROS adapter: the controller must find its interfaces,
// activate, update, and survive a deactivate/activate cycle. This guards the
// Humble-specific behaviour that the core tests cannot see: interface lookup by
// joint name (Humble's get_name() returns "joint/interface") and interfaces
// being re-loaned on every activation.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "acker_diff_controller/acker_diff_controller.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
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
