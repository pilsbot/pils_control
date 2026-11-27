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
 * Author: Pascal Pieper
 */

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/clock.hpp"
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

CallbackReturn AckerDiffController::on_init()
{
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
    auto_declare<double>("pid.oIa", pid_params_.overshoot_integral_adaptation);

    auto_declare<std::string>("odom_frame_id", odom_params_.odom_frame_id);
    auto_declare<std::string>("base_frame_id", odom_params_.base_frame_id);
    auto_declare<std::vector<double>>("pose_covariance_diagonal", std::vector<double>());
    auto_declare<std::vector<double>>("twist_covariance_diagonal", std::vector<double>());
    auto_declare<bool>("open_loop", odom_params_.open_loop);
    auto_declare<bool>("enable_odom_tf", odom_params_.enable_odom_tf);

    auto_declare<double>("cmd_vel_timeout", cmd_vel_timeout_.count() / 1000.0);
    auto_declare<bool>("publish_limited_velocity", publish_limited_velocity_);
    auto_declare<int>("velocity_rolling_window_size", 10);
    auto_declare<double>("invalid_input_grace_period", invalid_input_grace_period_);
    auto_declare<bool>("use_stamped_vel", use_stamped_vel_);

    auto_declare<bool>  ("linear.x.has_velocity_limits", false);
    auto_declare<bool>  ("linear.x.has_acceleration_limits", false);
    auto_declare<bool>  ("linear.x.has_jerk_limits", false);
    auto_declare<double>("linear.x.max_velocity", NAN);
    auto_declare<double>("linear.x.min_velocity", NAN);
    auto_declare<double>("linear.x.max_acceleration", NAN);
    auto_declare<double>("linear.x.min_acceleration", NAN);
    auto_declare<double>("linear.x.min_jerk", NAN);
    auto_declare<double>("linear.x.max_jerk", NAN);

    auto_declare<bool>  ("angle.z.has_position_limits", true);
    auto_declare<double>("angle.z.max_angle", 1.14);  // about 90 degrees
    auto_declare<double>("angle.z.min_angle", NAN);
    auto_declare<double>("angle.z.max_angular_velocity", 1.);
    auto_declare<double>("angle.z.max_angular_acceleration", 6.);
    auto_declare<double>("angle.z.max_angle_setpoint_diff", .85); // about 50 degrees
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
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

controller_interface::return_type AckerDiffController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto logger = get_node()->get_logger();


  std::shared_ptr<AckermannStamped> last_msg;
  received_command_msg_ptr_.get(last_msg);

  if (last_msg == nullptr)
  {
    RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }


  // Stamps use the controller manager's time (ROS time), not a steady clock,
  // so odometry and TF line up with the rest of the system.
  const auto & current_time = time;
  const auto & update_dt = period;

  AckerDiffCore::Measurement measurement{
    registered_steering_axle_handle_[0].get().get_value(), WheelPair{0.0, 0.0}};
  if (!odom_params_.open_loop)
  {
    for (size_t index = 0; index < wheel_params_.wheels_per_side; ++index)
    {
      measurement.wheel_position.left +=
        registered_left_wheel_handles_[index].position.get().get_value();
      measurement.wheel_position.right +=
        registered_right_wheel_handles_[index].position.get().get_value();
    }
    measurement.wheel_position.left /= wheel_params_.wheels_per_side;
    measurement.wheel_position.right /= wheel_params_.wheels_per_side;
  }

  AckerCommand command{
    last_msg->drive.speed, last_msg->drive.steering_angle,
    last_msg->drive.steering_angle_velocity};
  // Stop hard if cmd_vel has timed out (e.g. gamepad link loss): no ramp-down
  // at linear.x.max_acceleration, and the steering loop goes passive. The
  // stored message is shared with the subscriber callback, so leave it alone.
  // The timeout is latched per message (pointer and stamp): if ROS time steps
  // backwards, a timed-out command must not become fresh again. Only a new
  // message, or a new stamp on the stored one, lifts it.
  const rclcpp::Time stamp(last_msg->header.stamp);
  const bool latched = last_msg == timed_out_msg_ && stamp == timed_out_stamp_;
  const auto message_dt = get_node()->get_clock()->now() - stamp;
  if (latched || message_dt > cmd_vel_timeout_)
  {
    timed_out_msg_ = last_msg;
    timed_out_stamp_ = stamp;
    command.speed = 0;
    command.steering_angle_velocity = 0;
    command.hard_stop = true;
  }
  const auto out = core_.step(command, measurement, update_dt.seconds(), current_time.seconds());

  if (out.status == AckerDiffCore::Status::invalid_steering_angle) {
    RCLCPP_ERROR_THROTTLE(logger, *get_node()->get_clock(), 1000,
        "Could not get current steering angle! Halting the wheels.");
    // Humble's controller manager ignores the return value: stop here. The core
    // decides whether the ramp survives (invalid_input_grace_period).
    halt_wheels();
    publish_limited_command(current_time, out);
    return controller_interface::return_type::ERROR;
  }

  if (out.steering_limit_reset) {
    RCLCPP_WARN_THROTTLE(logger, *get_node()->get_clock(), 1000,
        "Current steering angle (%lf) over configured limit (%lf - %lf)!\n"
        "Resetting PID-Controller.\n"
        "If this happens frequently, consider re-tuning the PID values.",
        measurement.steering_angle, steering_params_.min_angle, steering_params_.max_angle);
  }
  if (out.linear_stopped) {
    RCLCPP_DEBUG(logger,
        "Steering error '%lf' rad exceeds max_angle_setpoint_diff '%lf' rad! "
        "Stopping linear motion.", out.steering_error,
        steering_params_.limit_linear_at_angle_diff_to_setpoint
    );
  }

  publish_limited_command(current_time, out);

  if (out.status == AckerDiffCore::Status::invalid_wheel_position) {
    RCLCPP_ERROR_THROTTLE(logger, *get_node()->get_clock(), 1000,
        "Either the left or right wheel position is invalid! Halting the wheels.");
    halt_wheels();
    return controller_interface::return_type::ERROR;
  }

  publish_odometry(current_time);

  // Set wheels turning rates:
  for (size_t index = 0; index < wheel_params_.wheels_per_side; ++index)
  {
    registered_left_wheel_handles_[index].velocity.get().set_value(out.wheel_rates.left);
    registered_right_wheel_handles_[index].velocity.get().set_value(out.wheel_rates.right);
  }

  return controller_interface::return_type::OK;
}

CallbackReturn AckerDiffController::on_configure(const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  // update parameters
  left_wheel_names_ = get_node()->get_parameter("left_wheel_names").as_string_array();
  right_wheel_names_ = get_node()->get_parameter("right_wheel_names").as_string_array();
  steering_axle_name_ = get_node()->get_parameter("steering_axle_name").as_string();

  RCLCPP_INFO_ONCE(
    logger,
    "Wheel names: \n%s\n%s\nSteering axle name:\n%s",
    left_wheel_names_.empty() ? "" : left_wheel_names_[0].c_str(),
    right_wheel_names_.empty() ? "" : right_wheel_names_[0].c_str(),
    steering_axle_name_.c_str());

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


  wheel_params_.separation = get_node()->get_parameter("wheel_separation").as_double();
  if(std::isnan(wheel_params_.separation) || wheel_params_.separation == 0) {
    RCLCPP_ERROR(logger,
        "Wheel separation '%lf' is unplausible!", wheel_params_.separation);
    return CallbackReturn::ERROR;
  }
  wheel_params_.wheelbase = get_node()->get_parameter("wheel_base_distance").as_double();
  if(std::isnan(wheel_params_.wheelbase) || wheel_params_.wheelbase == 0) {
    RCLCPP_ERROR(logger,
        "Wheel wheelbase distance '%lf' is unplausible!", wheel_params_.wheelbase);
    return CallbackReturn::ERROR;
  }
  wheel_params_.wheels_per_side =
    static_cast<size_t>(get_node()->get_parameter("wheels_per_side").as_int());
  wheel_params_.radius = get_node()->get_parameter("wheel_radius").as_double();
  wheel_params_.separation_multiplier =
    get_node()->get_parameter("wheel_separation_multiplier").as_double();
  wheel_params_.left_radius_multiplier =
    get_node()->get_parameter("left_wheel_radius_multiplier").as_double();
  wheel_params_.right_radius_multiplier =
    get_node()->get_parameter("right_wheel_radius_multiplier").as_double();

  velocity_rolling_window_size_ =
    static_cast<size_t>(get_node()->get_parameter("velocity_rolling_window_size").as_int());

  odom_params_.odom_frame_id = get_node()->get_parameter("odom_frame_id").as_string();
  odom_params_.base_frame_id = get_node()->get_parameter("base_frame_id").as_string();

  // Each diagonal has 6 variances >= 0 (x, y, z, roll, pitch, yaw); unset means all 0.
  const auto read_diagonal =
    [this, &logger](const std::string & name, std::array<double, 6> & diagonal) {
      const auto values = get_node()->get_parameter(name).as_double_array();
      if (values.empty()) {
        diagonal.fill(0.0);
        return true;
      }
      if (values.size() != diagonal.size()) {
        RCLCPP_ERROR(logger, "'%s' needs %zu entries, got %zu",
          name.c_str(), diagonal.size(), values.size());
        return false;
      }
      for (const double value : values) {
        if (!(value >= 0.0)) {
          RCLCPP_ERROR(logger, "'%s' entries must be >= 0, got %lf", name.c_str(), value);
          return false;
        }
      }
      std::copy(values.begin(), values.end(), diagonal.begin());
      return true;
    };
  if (!read_diagonal("pose_covariance_diagonal", odom_params_.pose_covariance_diagonal) ||
    !read_diagonal("twist_covariance_diagonal", odom_params_.twist_covariance_diagonal))
  {
    return CallbackReturn::ERROR;
  }

  odom_params_.open_loop = get_node()->get_parameter("open_loop").as_bool();
  odom_params_.enable_odom_tf = get_node()->get_parameter("enable_odom_tf").as_bool();

  cmd_vel_timeout_ = std::chrono::milliseconds{
    static_cast<int>(get_node()->get_parameter("cmd_vel_timeout").as_double() * 1000.0)};
  publish_limited_velocity_ = get_node()->get_parameter("publish_limited_velocity").as_bool();
  invalid_input_grace_period_ =
    get_node()->get_parameter("invalid_input_grace_period").as_double();
  if (!(invalid_input_grace_period_ >= 0.0)) {
    RCLCPP_ERROR(logger, "invalid_input_grace_period '%lf' must be >= 0 s",
      invalid_input_grace_period_);
    return CallbackReturn::ERROR;
  }
  use_stamped_vel_ = get_node()->get_parameter("use_stamped_vel").as_bool();

  try
  {
    limiter_linear_ = SpeedLimiter(
      get_node()->get_parameter("linear.x.has_velocity_limits").as_bool(),
      get_node()->get_parameter("linear.x.has_acceleration_limits").as_bool(),
      get_node()->get_parameter("linear.x.has_jerk_limits").as_bool(),
      get_node()->get_parameter("linear.x.min_velocity").as_double(),
      get_node()->get_parameter("linear.x.max_velocity").as_double(),
      get_node()->get_parameter("linear.x.min_acceleration").as_double(),
      get_node()->get_parameter("linear.x.max_acceleration").as_double(),
      get_node()->get_parameter("linear.x.min_jerk").as_double(),
      get_node()->get_parameter("linear.x.max_jerk").as_double());
  }
  catch (const std::runtime_error & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error configuring linear speed limiter: %s", e.what());
  }

  steering_params_ = SteeringParams {
      .has_position_limit =
        get_node()->get_parameter("angle.z.has_position_limits").as_bool(),
      .max_angle =
        get_node()->get_parameter("angle.z.max_angle").as_double(),
      .min_angle =
        get_node()->get_parameter("angle.z.min_angle").as_double(),
      .max_angular_velocity =
        get_node()->get_parameter("angle.z.max_angular_velocity").as_double(),
      .max_angular_acceleration =
        get_node()->get_parameter("angle.z.max_angular_acceleration").as_double(),
      .limit_linear_at_angle_diff_to_setpoint =
        get_node()->get_parameter("angle.z.max_angle_setpoint_diff").as_double(),
  };
  if(steering_params_.has_position_limit && isnan(steering_params_.min_angle)) {
    steering_params_.min_angle = -steering_params_.max_angle;
  }
  try
  {
    limiter_angle_ = SpeedLimiter(
      steering_params_.has_position_limit,
      !std::isnan(steering_params_.max_angular_velocity), false,
      steering_params_.min_angle,
      steering_params_.max_angle,
      NAN, steering_params_.max_angular_velocity);
  }
  catch (const std::runtime_error & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "max_angular_velocity: %lf", steering_params_.max_angular_velocity);
    RCLCPP_ERROR(get_node()->get_logger(), "Error configuring steering angle limiter: %s", e.what());
  }

  pid_params_ = PID::Settings{
    .Kp = get_node()->get_parameter("pid.Kp").as_double(),
    .Ki = get_node()->get_parameter("pid.Ki").as_double(),
    .Kd = get_node()->get_parameter("pid.Kd").as_double(),
    .max = steering_params_.max_angular_velocity,
    .min = -steering_params_.max_angular_velocity,
    .max_dv = steering_params_.max_angular_acceleration,
    .overshoot_integral_adaptation = get_node()->get_parameter("pid.oIa").as_double(),
  };

  RCLCPP_INFO(logger, "PID: p %.4lf i %.4lf d %.4lf", pid_params_.Kp, pid_params_.Ki, pid_params_.Kd);
  RCLCPP_INFO(logger, "PID: max %.4lf min %.4lf max_dv %.4lf", pid_params_.max, pid_params_.min, pid_params_.max_dv);
  RCLCPP_INFO(logger, "PID: oIa %.4lf", pid_params_.overshoot_integral_adaptation);

  core_.configure(AckerDiffCore::Config{
    WheelGeometry{
      wheel_params_.separation, wheel_params_.radius, wheel_params_.wheelbase,
      wheel_params_.separation_multiplier, wheel_params_.left_radius_multiplier,
      wheel_params_.right_radius_multiplier},
    steering_params_, pid_params_, limiter_linear_, limiter_angle_,
    odom_params_.open_loop, velocity_rolling_window_size_, invalid_input_grace_period_});

  if (!reset())
  {
    return CallbackReturn::ERROR;
  }

  // left and right sides are both equal at this point
  wheel_params_.wheels_per_side = left_wheel_names_.size();

  if (publish_limited_velocity_)
  {
    limited_command_publisher_ =
      get_node()->create_publisher<AckermannStamped>(DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_limited_command_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<AckermannStamped>>(limited_command_publisher_);
  }

  const AckermannStamped empty_cmd;
  received_command_msg_ptr_.set(std::make_shared<AckermannStamped>(empty_cmd));


  // initialize command subscriber
  if (use_stamped_vel_)
  {
    command_subscriber_ = get_node()->create_subscription<AckermannStamped>(
      DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<AckermannStamped> msg) -> void {
        if (!subscriber_is_active_)
        {
          RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }
        if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
        {
          RCLCPP_WARN_ONCE(
            get_node()->get_logger(),
            "Received AckermannStamped with zero timestamp, setting it to current "
            "time, this message will only be shown once");
          msg->header.stamp = get_node()->get_clock()->now();
        }
        received_command_msg_ptr_.set(std::move(msg));
      });
  }
  else
  {
    command_unstamped_subscriber_ = get_node()->create_subscription<Ackermann>(
      DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<Ackermann> msg) -> void {
        if (!subscriber_is_active_)
        {
          RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }

        // Store a new stamped command; the stored one may be read by update() right now.
        auto command_stamped = std::make_shared<AckermannStamped>();
        command_stamped->drive = *msg;
        command_stamped->header.stamp = get_node()->get_clock()->now();
        received_command_msg_ptr_.set(std::move(command_stamped));
      });
  }

  // initialize odometry publisher and messasge
  odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
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
  odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
    DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_transform_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
      odometry_transform_publisher_);

  // keeping track of the odom to base frame transform only
  auto & odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
  odometry_transform_message.transforms.resize(1);
  odometry_transform_message.transforms.front().header.frame_id = odom_params_.odom_frame_id;
  odometry_transform_message.transforms.front().child_frame_id = odom_params_.base_frame_id;

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
      get_node()->get_logger(), "Either left wheel interfaces, right wheel interfaces are non existent");
    return CallbackReturn::ERROR;
  }

  if (registered_steering_axle_handle_.empty())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "steering axle interface %s is non existent", steering_axle_name_.c_str());
    return CallbackReturn::ERROR;
  }

  subscriber_is_active_ = true;

  RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn AckerDiffController::on_deactivate(const rclcpp_lifecycle::State &)
{
  subscriber_is_active_ = false;
  halt();
  // The controller manager releases the loaned interfaces after deactivation and
  // loans fresh ones on the next activation, so these references must not survive.
  registered_left_wheel_handles_.clear();
  registered_right_wheel_handles_.clear();
  registered_steering_axle_handle_.clear();
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
  core_.reset();

  registered_left_wheel_handles_.clear();
  registered_right_wheel_handles_.clear();
  registered_steering_axle_handle_.clear();

  subscriber_is_active_ = false;
  command_subscriber_.reset();
  command_unstamped_subscriber_.reset();

  received_command_msg_ptr_.set(nullptr);
  timed_out_msg_.reset();
  return true;
}

CallbackReturn AckerDiffController::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

void AckerDiffController::halt()
{
  core_.halt();
  halt_wheels();
}

void AckerDiffController::halt_wheels()
{
  const auto halt_side = [](auto & wheel_handles) {
    for (const auto & wheel_handle : wheel_handles)
    {
      wheel_handle.velocity.get().set_value(0.0);
    }
  };

  halt_side(registered_left_wheel_handles_);
  halt_side(registered_right_wheel_handles_);
}

void AckerDiffController::publish_limited_command(
  const rclcpp::Time & current_time, const AckerDiffCore::Output & out)
{
  if (!publish_limited_velocity_ || !realtime_limited_command_publisher_->trylock())
  {
    return;
  }
  // Publish what the wheels actually get: on invalid input they are halted.
  const bool halted = out.status != AckerDiffCore::Status::ok;
  auto & limited_velocity_command = realtime_limited_command_publisher_->msg_;
  limited_velocity_command.header.stamp = current_time;
  limited_velocity_command.drive.speed = halted ? 0.0 : out.limited_command.speed;
  limited_velocity_command.drive.steering_angle = out.limited_command.steering_angle;
  limited_velocity_command.drive.steering_angle_velocity =
    halted ? 0.0 : out.limited_command.steering_angle_velocity;
  realtime_limited_command_publisher_->unlockAndPublish();
}

CallbackReturn AckerDiffController::configure_side(
  const std::string & side, const std::vector<std::string> & wheel_names,
  std::vector<WheelHandle> & registered_handles)
{
  auto logger = get_node()->get_logger();

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
        return interface.get_prefix_name() == wheel_name &&
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
        return interface.get_prefix_name() == wheel_name &&
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
  auto logger = get_node()->get_logger();
  if (name.empty())
  {
    RCLCPP_ERROR(logger, "No steering joint name specified");
    return CallbackReturn::ERROR;
  }

  const auto steering_handle = std::find_if(
    state_interfaces_.cbegin(), state_interfaces_.cend(), [&name](const auto & interface) {
      return interface.get_prefix_name() == name &&
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

void AckerDiffController::publish_odometry(const rclcpp::Time & current_time)
{
  // The estimator tracks the front axle; publish the rear axle centre (docs/adr/0002).
  const AckerDiffCore::BaseState & base = core_.rear_axle_centre();

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, base.pose.heading);

  if (realtime_odometry_publisher_->trylock())
  {
    auto & odometry_message = realtime_odometry_publisher_->msg_;
    odometry_message.header.stamp = current_time;
    odometry_message.pose.pose.position.x = base.pose.x;
    odometry_message.pose.pose.position.y = base.pose.y;
    odometry_message.pose.pose.orientation.x = orientation.x();
    odometry_message.pose.pose.orientation.y = orientation.y();
    odometry_message.pose.pose.orientation.z = orientation.z();
    odometry_message.pose.pose.orientation.w = orientation.w();
    odometry_message.twist.twist.linear.x = base.twist.linear_x;
    odometry_message.twist.twist.linear.y = base.twist.linear_y;
    odometry_message.twist.twist.angular.z = base.twist.angular;
    realtime_odometry_publisher_->unlockAndPublish();
  }

  if (odom_params_.enable_odom_tf && realtime_odometry_transform_publisher_->trylock())
  {
    auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
    transform.header.stamp = current_time;
    transform.transform.translation.x = base.pose.x;
    transform.transform.translation.y = base.pose.y;
    transform.transform.rotation.x = orientation.x();
    transform.transform.rotation.y = orientation.y();
    transform.transform.rotation.z = orientation.z();
    transform.transform.rotation.w = orientation.w();
    realtime_odometry_transform_publisher_->unlockAndPublish();
  }
}
}  // namespace acker_drive_controller



#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  acker_diff_controller::AckerDiffController, controller_interface::ControllerInterface)
