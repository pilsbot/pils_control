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
    auto_declare<double>("wheel_base_distance", wheel_params_.wheelbase);
    auto_declare<int>("wheels_per_side", wheel_params_.wheels_per_side);
    auto_declare<double>("wheel_radius", wheel_params_.radius);
    auto_declare<double>("wheel_separation_multiplier", wheel_params_.separation_multiplier);
    auto_declare<double>("left_wheel_radius_multiplier", wheel_params_.left_radius_multiplier);
    auto_declare<double>("right_wheel_radius_multiplier", wheel_params_.right_radius_multiplier);

    auto_declare<double>("pid.Kp", pid_params_.Kp);
    auto_declare<double>("pid.Ki", pid_params_.Ki);
    auto_declare<double>("pid.Kd", pid_params_.Kd);

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

    auto_declare<bool>("angle.z.has_position_limits", false);
    auto_declare<double>("angle.z.max_angle", NAN);
    auto_declare<double>("angle.z.min_angle", NAN);

    auto_declare<bool>("angular.z.has_velocity_limits", false);
    auto_declare<bool>("angular.z.has_acceleration_limits", false);
    auto_declare<bool>("angular.z.has_jerk_limits", false);
    auto_declare<double>("angular.z.max_velocity", NAN);
    auto_declare<double>("angular.z.min_velocity", NAN);
    auto_declare<double>("angular.z.max_acceleration", pid_params_.max_dv);
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
  // Brake if cmd_vel has timed out, override the stored command
  if (dt > cmd_vel_timeout_)
  {
    last_msg->drive.speed = 0;
    last_msg->drive.steering_angle_velocity = 0;
  }

  // linear_command and angular_command may be limited further by SpeedLimit,
  double linear_command = last_msg->drive.speed;
  double angle_command = last_msg->drive.steering_angle;
  double angular_command = last_msg->drive.steering_angle_velocity;

  const auto update_dt = current_time - previous_update_timestamp_;
  previous_update_timestamp_ = current_time;

  auto & last_command = previous_commands_.back().drive;
  auto & second_to_last_command = previous_commands_.front().drive;
  limiter_linear_.limit(
    linear_command, last_command.speed, second_to_last_command.speed, update_dt.seconds());
  limiter_angle_.limit_velocity(angle_command); // actually limits position here, lol
  limiter_angular_.limit(
    angular_command, last_command.steering_angle_velocity, second_to_last_command.steering_angle_velocity, update_dt.seconds());
  previous_commands_.pop();
  previous_commands_.emplace(*last_msg);
  angular_command = std::abs(angular_command);  //angular is considered in the direction of steering error


  const double current_steering_angle = registered_steering_axle_handle_[0].get().get_value();
  if (std::isnan(current_steering_angle)) {
    RCLCPP_ERROR_THROTTLE(logger, *node_->get_clock(), 1000,
        "Could not get current steering angle!");
    return controller_interface::return_type::ERROR;
  }

  double angular_correction = 0;
  // PID controller
  if(angular_command > 0) {     // if steering angle velocity = 0, dont regulate
    if(last_command.steering_angle_velocity == 0) {
      // we transitioned from passive to active steering, reset PID
      pid_controller_ = PID();
    }
    pid_params_.dt = update_dt.seconds();
    // negative feedback, because sensor is not mathematically but logically oriented
    angular_correction =
        -pid_controller_.calculate(angle_command, current_steering_angle, pid_params_);

    // check for significant controller overshoot on maximum angle for safety
    if( (current_steering_angle >= steering_params_.max_angle && angular_correction < 0) ||
        (current_steering_angle <= steering_params_.min_angle && angular_correction > 0) ) {
      //we would oversteer into endstops
      RCLCPP_WARN_THROTTLE(logger, *node_->get_clock(), 1000,
          "Current steering angle (%lf) over configured limit (%lf - %lf)!\n"
          "Resetting PID-Controller.\n"
          "If this happens frequently, consider re-tuning the PID values.",
          current_steering_angle, steering_params_.min_angle, steering_params_.max_angle);
      angular_correction = 0;
      pid_controller_ = PID();  // resets internals
    }

    RCLCPP_DEBUG(logger,
        "Angle error: %lf\n"
        "correction: %lf", (angle_command - current_steering_angle), angular_correction
    );
  }

  // logic that reduces linear speed if error is too big
  if(std::abs(angular_correction) > angular_command) {
    linear_command = 0;
    RCLCPP_DEBUG(logger,
        "Difference too big! Stopping linear motion."
    );
  }

  // limit linear acceleration and angular speed
  limiter_angular_.limit_velocity(angular_correction);
  limiter_linear_.limit_velocity(linear_command);  // only needed if logic earlier would also increase speed
  limiter_linear_.limit_acceleration(
      linear_command, last_limited_linear_command_, update_dt.seconds());

  //    Publish limited velocity
  last_limited_linear_command_ = linear_command;
  if (publish_limited_velocity_ && realtime_limited_command_publisher_->trylock())
  {
    auto & limited_velocity_command = realtime_limited_command_publisher_->msg_;
    limited_velocity_command.header.stamp = current_time;
    limited_velocity_command.drive.speed = linear_command;
    limited_velocity_command.drive.steering_angle = angle_command;
    limited_velocity_command.drive.steering_angle_velocity = angular_command;
    realtime_limited_command_publisher_->unlockAndPublish();
  }

  WheelSpeeds linear_speed{linear_command, linear_command};

  //TODO: magic value where turning angle is so small that different speeds are not needed
  const double min_abs_turning_angle_for_ackermann = 0.05;
  if(std::abs(angle_command) > min_abs_turning_angle_for_ackermann) {
    linear_speed = calc_turning_speeds(linear_command, angle_command);
  }

  WheelTurningRate final_turning_rate = mix_linear_and_angular(linear_speed, angular_correction);

  // odometry updates
  if(!update_odometry(final_turning_rate, current_time)) {
    return controller_interface::return_type::ERROR;
  }

  // Set wheels turning rates:
  for (size_t index = 0; index < wheel_params_.wheels_per_side; ++index)
  {
    registered_left_wheel_handles_[index].velocity.get().set_value(final_turning_rate.left);
    registered_right_wheel_handles_[index].velocity.get().set_value(final_turning_rate.right);
  }

  return controller_interface::return_type::OK;
}

CallbackReturn AckerDiffController::on_configure(const rclcpp_lifecycle::State &)
{
  auto logger = node_->get_logger();

  // update parameters
  left_wheel_names_ = node_->get_parameter("left_wheel_names").as_string_array();
  right_wheel_names_ = node_->get_parameter("right_wheel_names").as_string_array();
  steering_axle_name_ = node_->get_parameter("steering_axle_name").as_string();

  RCLCPP_INFO_ONCE(
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
  wheel_params_.wheelbase = node_->get_parameter("wheel_base_distance").as_double();
  if(std::isnan(wheel_params_.wheelbase) || wheel_params_.wheelbase == 0) {
    RCLCPP_ERROR(logger,
        "Wheel wheelbase distance '%lf' is unplausible!", wheel_params_.wheelbase);
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

  steering_params_ = SteeringParams {
      .has_position_limit = node_->get_parameter("angle.z.has_position_limits").as_bool(),
      .max_angle = node_->get_parameter("angle.z.max_angle").as_double(),
      .min_angle = node_->get_parameter("angle.z.min_angle").as_double()
  };
  if(steering_params_.has_position_limit && isnan(steering_params_.min_angle)) {
    steering_params_.min_angle = -steering_params_.max_angle;
  }
  try
  {
    limiter_angle_ = SpeedLimiter(
      steering_params_.has_position_limit,
      false, false,
      steering_params_.min_angle,
      steering_params_.max_angle);
  }
  catch (const std::runtime_error & e)
  {
    RCLCPP_ERROR(node_->get_logger(), "Error configuring steering angle limiter: %s", e.what());
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

  pid_params_ = PID::Settings{
    .Kp = node_->get_parameter("pid.Kp").as_double(),
    .Ki = node_->get_parameter("pid.Ki").as_double(),
    .Kd = node_->get_parameter("pid.Kd").as_double(),
    .dt = 1,    //is set dynamically later by update rate
    .max = node_->get_parameter("angular.z.max_velocity").as_double(),
    .min = node_->get_parameter("angular.z.min_velocity").as_double(),
    .max_dv = NAN
  };

  if(node_->get_parameter("angular.z.has_acceleration_limits").as_bool()) {
    pid_params_.max_dv = node_->get_parameter("angular.z.max_acceleration").as_double();
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

  const AckermannStamped empty_cmd;
  received_command_msg_ptr_.set(std::make_shared<AckermannStamped>(empty_cmd));

  // Fill last two commands with default constructed commands
  previous_commands_.emplace(empty_cmd);
  previous_commands_.emplace(empty_cmd);

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
  pid_controller_ = PID();

  // release the old queue
  std::queue<AckermannStamped> empty;
  std::swap(previous_commands_, empty);

  registered_left_wheel_handles_.clear();
  registered_right_wheel_handles_.clear();
  registered_steering_axle_handle_.clear();

  last_limited_linear_command_ = 0;

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

  last_limited_linear_command_ = 0;

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
      WheelHandle{.position = std::ref(*state_handle),
                  .velocity = std::ref(*command_handle)
      });
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

  const auto steering_handle = std::find_if(
    state_interfaces_.cbegin(), state_interfaces_.cend(), [&name](const auto & interface) {
      return interface.get_name() == name &&
             interface.get_interface_name() == HW_IF_POSITION;
    });

  if (steering_handle == state_interfaces_.cend())
  {
    RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", name.c_str());
    return CallbackReturn::ERROR;
  }


  handle.emplace_back(std::ref(*steering_handle));

  return CallbackReturn::SUCCESS;
}

AckerDiffController::WheelSpeeds AckerDiffController::calc_turning_speeds(
    const double linear_speed, const double turning_angle) {
  WheelSpeeds ret{linear_speed, linear_speed};
  if (turning_angle == 0) {
    RCLCPP_WARN(node_->get_logger(), "calc_turning_speeds: Turning angle is zero!");
    return ret;
  }

  // get perhaps updated config
  const double wheel_separation = wheel_params_.separation_multiplier * wheel_params_.separation;
  const double wheel_base_distance = wheel_params_.wheelbase;

  auto abs_alpha = M_PI_2 - std::abs(turning_angle);
  auto turning_radius = wheel_base_distance / cos(abs_alpha);


  ret.left = (turning_radius + wheel_separation/2.0) * atan(linear_speed / turning_radius);
  ret.right = (turning_radius - wheel_separation/2.0) * atan(linear_speed / turning_radius);

  if(turning_angle < 0)
    std::swap(ret.left, ret.right);

  RCLCPP_DEBUG(node_->get_logger(),
      "\n\tlinear_part_left : %lf = (%lf + %lf/2) * atan(%lf / %lf)"
      "\n\tlinear_part_right: %lf = (%lf - %lf/2) * %lf",
      ret.left, turning_radius, wheel_separation, linear_speed, turning_radius,
      ret.right, turning_radius, wheel_separation, atan(linear_speed / turning_radius)
  );

  return ret;
}

AckerDiffController::WheelSpeeds AckerDiffController::mix_linear_and_angular(
    const AckerDiffController::WheelSpeeds& linear, const double angular_speed) {
  // get perhaps updated config
  const double wheel_separation =
      wheel_params_.separation_multiplier * wheel_params_.separation;
  const double left_wheel_radius =
      wheel_params_.left_radius_multiplier * wheel_params_.radius;
  const double right_wheel_radius =
      wheel_params_.right_radius_multiplier * wheel_params_.radius;

  WheelSpeeds ret {
    (linear.left - angular_speed * wheel_separation / 2.0) / left_wheel_radius,
    (linear.right + angular_speed * wheel_separation / 2.0) / right_wheel_radius
  };

  RCLCPP_DEBUG(node_->get_logger(),
     "Velocity left: %lf = (%lf - %lf * %lf / 2.0) / %lf",
      ret.left, linear.left , angular_speed, wheel_separation, left_wheel_radius);

  return ret;
}

bool AckerDiffController::update_odometry(
    const AckerDiffController::WheelTurningRate& set_turning_rate,
    const rclcpp::Time& current_time) {

  if (odom_params_.open_loop)
  {
    // get perhaps updated config
    const double wheel_separation =
        wheel_params_.separation_multiplier * wheel_params_.separation;
    const double left_wheel_radius =
        wheel_params_.left_radius_multiplier * wheel_params_.radius;
    const double right_wheel_radius =
        wheel_params_.right_radius_multiplier * wheel_params_.radius;

    //back-calculate wheel turning rates to linear/angular proportions
    WheelSpeeds set_velocity = {.left = set_turning_rate.left * left_wheel_radius,
                                .right = set_turning_rate.right * right_wheel_radius};

    // basically inverse of mix_linear_and_angular()
    double mean_linear = ( set_velocity.left + set_velocity.right ) / 2;

    double angular_proportion = (set_velocity.right - set_velocity.left)
                                / (wheel_separation);   // Why not divide by 2?
    odometry_.updateOpenLoop(mean_linear , angular_proportion, current_time);
  }
  else
  {
    double left_feedback_mean = 0.0;
    double right_feedback_mean = 0.0;
    for (size_t index = 0; index < wheel_params_.wheels_per_side; ++index)
    {
      const double left_feedback =
          registered_left_wheel_handles_[index].position.get().get_value();
      const double right_feedback =
          registered_right_wheel_handles_[index].position.get().get_value();

      if (std::isnan(left_feedback) || std::isnan(right_feedback))
      {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Either the left or right wheel is invalid for index [%zu]", index);
        return false;
      }

      left_feedback_mean += left_feedback;
      right_feedback_mean += right_feedback;
    }
    left_feedback_mean /= wheel_params_.wheels_per_side;
    right_feedback_mean /= wheel_params_.wheels_per_side;

    odometry_.update(left_feedback_mean, right_feedback_mean, current_time);

  }

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.getHeading());

  if (realtime_odometry_publisher_->trylock())
  {
    auto & odometry_message = realtime_odometry_publisher_->msg_;
    odometry_message.header.stamp = current_time;
    odometry_message.pose.pose.position.x = odometry_.getX();
    odometry_message.pose.pose.position.y = odometry_.getY();
    odometry_message.pose.pose.orientation.x = orientation.x();
    odometry_message.pose.pose.orientation.y = orientation.y();
    odometry_message.pose.pose.orientation.z = orientation.z();
    odometry_message.pose.pose.orientation.w = orientation.w();
    odometry_message.twist.twist.linear.x = odometry_.getLinear();
    odometry_message.twist.twist.angular.z = odometry_.getAngular();
    realtime_odometry_publisher_->unlockAndPublish();
  }

  if (odom_params_.enable_odom_tf && realtime_odometry_transform_publisher_->trylock())
  {
    auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
    transform.header.stamp = current_time;
    transform.transform.translation.x = odometry_.getX();
    transform.transform.translation.y = odometry_.getY();
    transform.transform.rotation.x = orientation.x();
    transform.transform.rotation.y = orientation.y();
    transform.transform.rotation.z = orientation.z();
    transform.transform.rotation.w = orientation.w();
    realtime_odometry_transform_publisher_->unlockAndPublish();
  }
  return true;
}
}  // namespace acker_drive_controller



#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  acker_diff_controller::AckerDiffController, controller_interface::ControllerInterface)
