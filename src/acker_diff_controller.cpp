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
 * Author: Bence Magyar, Enrique Fernández, Manuel Meraz
 */

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "acker_diff_controller/acker_diff_controller.hpp"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}  // namespace

namespace acker_diff_controller
{
using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

AckerDiffController::AckerDiffController() : controller_interface::ControllerInterface() {}

controller_interface::return_type AckerDiffController::init(const std::string & controller_name)
{
  // initialize lifecycle node
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK)
  {
    return ret;
  }

  try
  {
    // with the lifecycle node being initialized, we can declare parameters
    auto_declare<std::vector<std::string>>("left_wheel_names", std::vector<std::string>());
    auto_declare<std::vector<std::string>>("right_wheel_names", std::vector<std::string>());
    auto_declare<std::string>("steering_axle_name", std::string());

    auto_declare<double>("wheel_separation", wheel_params_.separation);
    auto_declare<int>("wheels_per_side", wheel_params_.wheels_per_side);
    auto_declare<double>("wheel_radius", wheel_params_.radius);
    auto_declare<double>("wheel_separation_multiplier", wheel_params_.separation_multiplier);
    auto_declare<double>("left_wheel_radius_multiplier", wheel_params_.left_radius_multiplier);
    auto_declare<double>("right_wheel_radius_multiplier", wheel_params_.right_radius_multiplier);

    auto_declare<std::string>("odom_frame_id", odom_params_.odom_frame_id);
    auto_declare<std::string>("base_frame_id", odom_params_.base_frame_id);
    auto_declare<std::vector<double>>("pose_covariance_diagonal", std::vector<double>());
    auto_declare<std::vector<double>>("twist_covariance_diagonal", std::vector<double>());
    auto_declare<bool>("open_loop", odom_params_.open_loop);
    auto_declare<bool>("enable_odom_tf", odom_params_.enable_odom_tf);

    auto_declare<double>("cmd_vel_timeout", cmd_vel_timeout_.count() / 1000.0);
    auto_declare<bool>("publish_limited_velocity", publish_limited_velocity_);
    auto_declare<int>("velocity_rolling_window_size", 10);
    auto_declare<bool>("use_stamped_vel", use_stamped_vel_);

    auto_declare<bool>("linear.x.has_velocity_limits", false);
    auto_declare<bool>("linear.x.has_acceleration_limits", false);
    auto_declare<bool>("linear.x.has_jerk_limits", false);
    auto_declare<double>("linear.x.max_velocity", NAN);
    auto_declare<double>("linear.x.min_velocity", NAN);
    auto_declare<double>("linear.x.max_acceleration", NAN);
    auto_declare<double>("linear.x.min_acceleration", NAN);
    auto_declare<double>("linear.x.max_jerk", NAN);
    auto_declare<double>("linear.x.min_jerk", NAN);

    //TODO: actually conform to these limits
    auto_declare<double>("angle.z.max_angle", NAN);
    auto_declare<double>("angle.z.min_angle", NAN);

    auto_declare<bool>("angular.z.has_velocity_limits", false);
    auto_declare<bool>("angular.z.has_acceleration_limits", false);
    auto_declare<bool>("angular.z.has_jerk_limits", false);
    auto_declare<double>("angular.z.max_velocity", NAN);
    auto_declare<double>("angular.z.min_velocity", NAN);
    auto_declare<double>("angular.z.max_acceleration", NAN);
    auto_declare<double>("angular.z.min_acceleration", NAN);
    auto_declare<double>("angular.z.max_jerk", NAN);
    auto_declare<double>("angular.z.min_jerk", NAN);
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

InterfaceConfiguration AckerDiffController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (const auto & joint_name : left_wheel_names_)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  for (const auto & joint_name : right_wheel_names_)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }

  // todo: is this the "output" interface?

  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration AckerDiffController::state_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (const auto & joint_name : left_wheel_names_)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
  }
  for (const auto & joint_name : right_wheel_names_)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
  }

  conf_names.push_back(steering_axle_name_ + "/" + HW_IF_POSITION);

  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type AckerDiffController::update()
{
  auto logger = node_->get_logger();
  RCLCPP_WARN_ONCE(
    logger,
    "update function called"
  );
  if (get_current_state().id() == State::PRIMARY_STATE_INACTIVE)
  {
    if (!is_halted)
    {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }

  const auto current_time = node_->get_clock()->now();

  std::shared_ptr<AckermannStamped> last_msg;
  received_command_msg_ptr_.get(last_msg);

  if (last_msg == nullptr)
  {
    RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  const auto dt = current_time - last_msg->header.stamp;
  // Brake if cmd_vel has timeout, override the stored command
  if (dt > cmd_vel_timeout_)
  {
    last_msg->drive.speed = 0;
    last_msg->drive.steering_angle_velocity = 0;
  }

  // linear_command and angular_command may be limited further by SpeedLimit,
  // without affecting the stored twist command
  double linear_command = last_msg->drive.speed;
  double angle_command = last_msg->drive.steering_angle;
  double angular_command = last_msg->drive.steering_angle_velocity;

  // Apply (possibly new) multipliers:
  const auto wheels = wheel_params_;
  const double wheel_separation = wheels.separation_multiplier * wheels.separation;
  if(std::isnan(wheel_separation))
    RCLCPP_ERROR(logger,"Wheel_separation is NaN!");
  const double left_wheel_radius = wheels.left_radius_multiplier * wheels.radius;
  if(std::isnan(left_wheel_radius))
    RCLCPP_ERROR(logger,"left_wheel_radius is NaN!");
  const double right_wheel_radius = wheels.right_radius_multiplier * wheels.radius;
  if(std::isnan(right_wheel_radius))
    RCLCPP_ERROR(logger,"right_wheel_radius is NaN!");

  const auto update_dt = current_time - previous_update_timestamp_;
  previous_update_timestamp_ = current_time;

  auto & last_command = previous_commands_.back().drive;
  auto & second_to_last_command = previous_commands_.front().drive;
  limiter_linear_.limit(
    linear_command, last_command.speed, second_to_last_command.speed, update_dt.seconds());
  limiter_angular_.limit(
    angular_command, last_command.steering_angle_velocity, second_to_last_command.steering_angle_velocity, update_dt.seconds());
  //todo: add steering angle limit as well
  previous_commands_.pop();
  previous_commands_.emplace(*last_msg);

  //    Publish limited velocity
  if (publish_limited_velocity_ && realtime_limited_command_publisher_->trylock())
  {
    auto & limited_velocity_command = realtime_limited_command_publisher_->msg_;
    limited_velocity_command.header.stamp = current_time;
    limited_velocity_command.drive.speed = linear_command;
    limited_velocity_command.drive.steering_angle_velocity = angular_command;
    //todo: add steering angle as well
    realtime_limited_command_publisher_->unlockAndPublish();
  }

  // TODO: statemachine with differences and shit
  const double current_steering_angle = registered_steering_axle_handle_[0].get().get_value();
  if (std::isnan(current_steering_angle)) {
    RCLCPP_ERROR(
      logger, "Could not get current steering angle!");
    return controller_interface::return_type::ERROR;
  }

  RCLCPP_WARN_ONCE(
    logger,
    "From current steering angle " + std::to_string(current_steering_angle) +
    "\ncommands:" +
    "\n\tlinear:  " + std::to_string(linear_command) +
    "\n\tangle:   " + std::to_string(angle_command) +
    "\n\tangular: " + std::to_string(angular_command)
  );

  // Compute wheels velocities:
  (void) angle_command;
  const double velocity_left =
    (linear_command - angular_command * wheel_separation / 2.0) / left_wheel_radius;
  RCLCPP_INFO_ONCE(logger,
      "%lf = (%lf - %lf * %lf / 2.0) / %lf",
      velocity_left, linear_command, angular_command, wheel_separation, left_wheel_radius);
  const double velocity_right =
    (linear_command + angular_command * wheel_separation / 2.0) / right_wheel_radius;

  // Set wheels velocities:
  for (size_t index = 0; index < wheels.wheels_per_side; ++index)
  {
    RCLCPP_WARN_ONCE(
      logger,
        "setting velocity: \n" +
        std::to_string(velocity_left) + "\n" +
        std::to_string(velocity_right)
    );
    registered_left_wheel_handles_[index].velocity.get().set_value(velocity_left);
    registered_right_wheel_handles_[index].velocity.get().set_value(velocity_right);
  }

  return controller_interface::return_type::OK;
}

CallbackReturn AckerDiffController::on_configure(const rclcpp_lifecycle::State &)
{
  auto logger = node_->get_logger();
  RCLCPP_WARN(
    logger,
    "on_configure called"
  );

  // update parameters
  left_wheel_names_ = node_->get_parameter("left_wheel_names").as_string_array();
  right_wheel_names_ = node_->get_parameter("right_wheel_names").as_string_array();
  steering_axle_name_ = node_->get_parameter("steering_axle_name").as_string();

  RCLCPP_WARN_ONCE(
    logger,
    "Wheel names: \n" +
    left_wheel_names_[0] + "\n" +
    right_wheel_names_[0] + "\nSteering axle name:\n" +
    steering_axle_name_
  );

  if (left_wheel_names_.size() != right_wheel_names_.size())
  {
    RCLCPP_ERROR(
      logger, "The number of left wheels [%zu] and the number of right wheels [%zu] are different",
      left_wheel_names_.size(), right_wheel_names_.size());
    return CallbackReturn::ERROR;
  }

  if (left_wheel_names_.empty() || right_wheel_names_.empty())
  {
    RCLCPP_ERROR(logger, "Wheel names parameters are empty!");
    return CallbackReturn::ERROR;
  }

  if( steering_axle_name_.empty()) {
    RCLCPP_ERROR(
      logger, "Steering axle is not configured!");
    return CallbackReturn::ERROR;
  }


  wheel_params_.separation = node_->get_parameter("wheel_separation").as_double();
  if(std::isnan(wheel_params_.separation) || wheel_params_.separation == 0) {
    RCLCPP_ERROR(logger,
        "Wheel separation '%lf' is unplausible!", wheel_params_.separation);
    return CallbackReturn::ERROR;
  }
  wheel_params_.wheels_per_side =
    static_cast<size_t>(node_->get_parameter("wheels_per_side").as_int());
  wheel_params_.radius = node_->get_parameter("wheel_radius").as_double();
  wheel_params_.separation_multiplier =
    node_->get_parameter("wheel_separation_multiplier").as_double();
  wheel_params_.left_radius_multiplier =
    node_->get_parameter("left_wheel_radius_multiplier").as_double();
  wheel_params_.right_radius_multiplier =
    node_->get_parameter("right_wheel_radius_multiplier").as_double();

  const auto wheels = wheel_params_;

  const double wheel_separation = wheels.separation_multiplier * wheels.separation;
  const double left_wheel_radius = wheels.left_radius_multiplier * wheels.radius;
  const double right_wheel_radius = wheels.right_radius_multiplier * wheels.radius;

  odometry_.setWheelParams(wheel_separation, left_wheel_radius, right_wheel_radius);
  odometry_.setVelocityRollingWindowSize(
    node_->get_parameter("velocity_rolling_window_size").as_int());

  odom_params_.odom_frame_id = node_->get_parameter("odom_frame_id").as_string();
  odom_params_.base_frame_id = node_->get_parameter("base_frame_id").as_string();

  auto pose_diagonal = node_->get_parameter("pose_covariance_diagonal").as_double_array();
  std::copy(
    pose_diagonal.begin(), pose_diagonal.end(), odom_params_.pose_covariance_diagonal.begin());

  auto twist_diagonal = node_->get_parameter("twist_covariance_diagonal").as_double_array();
  std::copy(
    twist_diagonal.begin(), twist_diagonal.end(), odom_params_.twist_covariance_diagonal.begin());

  odom_params_.open_loop = node_->get_parameter("open_loop").as_bool();
  odom_params_.enable_odom_tf = node_->get_parameter("enable_odom_tf").as_bool();

  cmd_vel_timeout_ = std::chrono::milliseconds{
    static_cast<int>(node_->get_parameter("cmd_vel_timeout").as_double() * 1000.0)};
  publish_limited_velocity_ = node_->get_parameter("publish_limited_velocity").as_bool();
  use_stamped_vel_ = node_->get_parameter("use_stamped_vel").as_bool();

  try
  {
    limiter_linear_ = SpeedLimiter(
      node_->get_parameter("linear.x.has_velocity_limits").as_bool(),
      node_->get_parameter("linear.x.has_acceleration_limits").as_bool(),
      node_->get_parameter("linear.x.has_jerk_limits").as_bool(),
      node_->get_parameter("linear.x.min_velocity").as_double(),
      node_->get_parameter("linear.x.max_velocity").as_double(),
      node_->get_parameter("linear.x.min_acceleration").as_double(),
      node_->get_parameter("linear.x.max_acceleration").as_double(),
      node_->get_parameter("linear.x.min_jerk").as_double(),
      node_->get_parameter("linear.x.max_jerk").as_double());
  }
  catch (const std::runtime_error & e)
  {
    RCLCPP_ERROR(node_->get_logger(), "Error configuring linear speed limiter: %s", e.what());
  }

  try
  {
    limiter_angular_ = SpeedLimiter(
      node_->get_parameter("angular.z.has_velocity_limits").as_bool(),
      node_->get_parameter("angular.z.has_acceleration_limits").as_bool(),
      node_->get_parameter("angular.z.has_jerk_limits").as_bool(),
      node_->get_parameter("angular.z.min_velocity").as_double(),
      node_->get_parameter("angular.z.max_velocity").as_double(),
      node_->get_parameter("angular.z.min_acceleration").as_double(),
      node_->get_parameter("angular.z.max_acceleration").as_double(),
      node_->get_parameter("angular.z.min_jerk").as_double(),
      node_->get_parameter("angular.z.max_jerk").as_double());
  }
  catch (const std::runtime_error & e)
  {
    RCLCPP_ERROR(node_->get_logger(), "Error configuring angular speed limiter: %s", e.what());
  }

  if (!reset())
  {
    return CallbackReturn::ERROR;
  }

  // left and right sides are both equal at this point
  wheel_params_.wheels_per_side = left_wheel_names_.size();

  if (publish_limited_velocity_)
  {
    limited_command_publisher_ =
      node_->create_publisher<AckermannStamped>(DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_limited_command_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<AckermannStamped>>(limited_command_publisher_);
  }

  const AckermannStamped empty_twist;
  received_command_msg_ptr_.set(std::make_shared<AckermannStamped>(empty_twist));

  // Fill last two commands with default constructed commands
  previous_commands_.emplace(empty_twist);
  previous_commands_.emplace(empty_twist);

  // initialize command subscriber
  if (use_stamped_vel_)
  {
    command_subscriber_ = node_->create_subscription<AckermannStamped>(
      DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<AckermannStamped> msg) -> void {
        if (!subscriber_is_active_)
        {
          RCLCPP_WARN(node_->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }
        if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
        {
          RCLCPP_WARN_ONCE(
            node_->get_logger(),
            "Received AckermannStamped with zero timestamp, setting it to current "
            "time, this message will only be shown once");
          msg->header.stamp = node_->get_clock()->now();
        }
        received_command_msg_ptr_.set(std::move(msg));
      });
  }
  else
  {
    command_unstamped_subscriber_ = node_->create_subscription<Ackermann>(
      DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<Ackermann> msg) -> void {
        if (!subscriber_is_active_)
        {
          RCLCPP_WARN(node_->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }

        // Write fake header in the stored stamped command
        std::shared_ptr<AckermannStamped> command_stamped;
        received_command_msg_ptr_.get(command_stamped);
        command_stamped->drive = *msg;
        command_stamped->header.stamp = node_->get_clock()->now();
      });
  }

  // initialize odometry publisher and messasge
  odometry_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>(
    DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      odometry_publisher_);

  auto & odometry_message = realtime_odometry_publisher_->msg_;
  odometry_message.header.frame_id = odom_params_.odom_frame_id;
  odometry_message.child_frame_id = odom_params_.base_frame_id;

  // initialize odom values zeros
  odometry_message.twist =
    geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

  constexpr size_t NUM_DIMENSIONS = 6;
  for (size_t index = 0; index < 6; ++index)
  {
    // 0, 7, 14, 21, 28, 35
    const size_t diagonal_index = NUM_DIMENSIONS * index + index;
    odometry_message.pose.covariance[diagonal_index] = odom_params_.pose_covariance_diagonal[index];
    odometry_message.twist.covariance[diagonal_index] =
      odom_params_.twist_covariance_diagonal[index];
  }

  // initialize transform publisher and message
  odometry_transform_publisher_ = node_->create_publisher<tf2_msgs::msg::TFMessage>(
    DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_transform_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
      odometry_transform_publisher_);

  // keeping track of odom and base_link transforms only
  auto & odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
  odometry_transform_message.transforms.resize(1);
  odometry_transform_message.transforms.front().header.frame_id = odom_params_.odom_frame_id;
  odometry_transform_message.transforms.front().child_frame_id = odom_params_.base_frame_id;

  previous_update_timestamp_ = node_->get_clock()->now();
  return CallbackReturn::SUCCESS;
}

CallbackReturn AckerDiffController::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    node_->get_logger(),
    "on_activate called"
  );

  const auto left_result =
    configure_side("left", left_wheel_names_, registered_left_wheel_handles_);
  const auto right_result =
    configure_side("right", right_wheel_names_, registered_right_wheel_handles_);
  const auto steering_angle_result =
    configure_steering_angle(steering_axle_name_, registered_steering_axle_handle_);

  if (left_result == CallbackReturn::ERROR || right_result == CallbackReturn::ERROR
      || steering_angle_result == CallbackReturn::ERROR)
  {
    return CallbackReturn::ERROR;
  }

  if (registered_left_wheel_handles_.empty() || registered_right_wheel_handles_.empty())
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Either left wheel interfaces, right wheel interfaces are non existent");
    return CallbackReturn::ERROR;
  }

  if (registered_steering_axle_handle_.empty())
  {
    RCLCPP_ERROR(
      node_->get_logger(), "steering axle interface ", steering_axle_name_," is non existent");
    return CallbackReturn::ERROR;
  }

  is_halted = false;
  subscriber_is_active_ = true;

  RCLCPP_DEBUG(node_->get_logger(), "Subscriber and publisher are now active.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn AckerDiffController::on_deactivate(const rclcpp_lifecycle::State &)
{
  subscriber_is_active_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn AckerDiffController::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return CallbackReturn::ERROR;
  }

  received_command_msg_ptr_.set(std::make_shared<AckermannStamped>());
  return CallbackReturn::SUCCESS;
}

CallbackReturn AckerDiffController::on_error(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

bool AckerDiffController::reset()
{
  odometry_.resetOdometry();

  // release the old queue
  std::queue<AckermannStamped> empty;
  std::swap(previous_commands_, empty);

  registered_left_wheel_handles_.clear();
  registered_right_wheel_handles_.clear();
  registered_steering_axle_handle_.clear();

  subscriber_is_active_ = false;
  command_subscriber_.reset();
  command_unstamped_subscriber_.reset();

  received_command_msg_ptr_.set(nullptr);
  is_halted = false;
  return true;
}

CallbackReturn AckerDiffController::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

void AckerDiffController::halt()
{
  const auto halt_wheels = [](auto & wheel_handles) {
    for (const auto & wheel_handle : wheel_handles)
    {
      wheel_handle.velocity.get().set_value(0.0);
    }
  };

  halt_wheels(registered_left_wheel_handles_);
  halt_wheels(registered_right_wheel_handles_);
}

CallbackReturn AckerDiffController::configure_side(
  const std::string & side, const std::vector<std::string> & wheel_names,
  std::vector<WheelHandle> & registered_handles)
{
  auto logger = node_->get_logger();

  if (wheel_names.empty())
  {
    RCLCPP_ERROR(logger, "No '%s' wheel names specified", side.c_str());
    return CallbackReturn::ERROR;
  }

  // register handles
  registered_handles.reserve(wheel_names.size());
  for (const auto & wheel_name : wheel_names)
  {
    const auto state_handle = std::find_if(
      state_interfaces_.cbegin(), state_interfaces_.cend(), [&wheel_name](const auto & interface) {
        return interface.get_name() == wheel_name &&
               interface.get_interface_name() == HW_IF_POSITION;
      });

    if (state_handle == state_interfaces_.cend())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", wheel_name.c_str());
      return CallbackReturn::ERROR;
    }

    const auto command_handle = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&wheel_name](const auto & interface) {
        return interface.get_name() == wheel_name &&
               interface.get_interface_name() == HW_IF_VELOCITY;
      });

    if (command_handle == command_interfaces_.end())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
      return CallbackReturn::ERROR;
    }

    registered_handles.emplace_back(
      WheelHandle{std::ref(*state_handle), std::ref(*command_handle)});
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn AckerDiffController::configure_steering_angle(std::string & name,
    std::vector<SteeringHandle>& handle) {
  auto logger = node_->get_logger();
  if (name.empty())
  {
    RCLCPP_ERROR(logger, "No steering joint name specified");
    return CallbackReturn::ERROR;
  }

  const auto state_handle = std::find_if(
    state_interfaces_.cbegin(), state_interfaces_.cend(), [&name](const auto & interface) {
      return interface.get_name() == name &&
             interface.get_interface_name() == HW_IF_POSITION;
    });

  if (state_handle == state_interfaces_.cend())
  {
    RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", name.c_str());
    return CallbackReturn::ERROR;
  }


  handle.emplace_back(std::ref(*state_handle));

  return CallbackReturn::SUCCESS;
}

}  // namespace acker_drive_controller



#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  acker_diff_controller::AckerDiffController, controller_interface::ControllerInterface)
