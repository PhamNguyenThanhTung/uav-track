/****************************************************************************
 *
 * Copyright (c) 2026 UAV Project.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * ROS 2 Altitude Controller for UAV using PX4 Offboard Mode
 *
 ****************************************************************************/

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

using namespace std::chrono_literals;

class AltitudeControlNode : public rclcpp::Node
{
public:
  AltitudeControlNode()
  : Node("altitude_control")
  {
    // Declare parameters with default values
    this->declare_parameter<double>("target_altitude_m", 2.0);
    this->declare_parameter<double>("publish_rate_hz", 10.0);

    target_altitude_m_ = this->get_parameter("target_altitude_m").as_double();
    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();

    if (!is_publish_rate_valid(publish_rate_hz_)) {
      RCLCPP_FATAL(this->get_logger(),
                   "Invalid publish_rate_hz: %f. Must produce a finite, representable period.",
                   publish_rate_hz_);
      throw std::invalid_argument("publish_rate_hz must produce a finite, representable period");
    }

    RCLCPP_INFO(this->get_logger(), "Starting AltitudeControlNode");
    RCLCPP_INFO(this->get_logger(), "Target altitude: %.2f m", target_altitude_m_);
    RCLCPP_INFO(this->get_logger(), "Publish rate: %.2f Hz", publish_rate_hz_);

    // Register parameter callback for dynamic runtime updates
    param_callback_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        double requested_target_altitude = target_altitude_m_;
        double requested_publish_rate = publish_rate_hz_;
        bool target_altitude_changed = false;
        bool publish_rate_changed = false;

        for (const auto & param : parameters) {
          if (param.get_name() == "target_altitude_m") {
            requested_target_altitude = param.as_double();
            target_altitude_changed = true;
          } else if (param.get_name() == "publish_rate_hz") {
            requested_publish_rate = param.as_double();
            if (!is_publish_rate_valid(requested_publish_rate)) {
              result.successful = false;
              result.reason = "publish_rate_hz must produce a finite, representable period";
              return result;
            }
            publish_rate_changed = true;
          }
        }

        if (target_altitude_changed) {
          target_altitude_m_ = requested_target_altitude;
          RCLCPP_INFO(this->get_logger(), "Target altitude updated to: %.2f m", target_altitude_m_);
        }
        if (publish_rate_changed) {
          // The node uses rclcpp::spin's single-threaded executor. Defer timer
          // replacement to the timer callback so the parameter service never
          // mutates the timer that may currently be executing.
          pending_publish_rate_hz_.store(requested_publish_rate, std::memory_order_release);
          RCLCPP_INFO(this->get_logger(),
                      "Publish rate update queued: %.2f Hz (applies on the next control tick)",
                      requested_publish_rate);
        }
        return result;
      });

    // QoS for PX4:
    // Publications to FMU: best effort, volatile durability
    rclcpp::QoS pub_qos(10);
    pub_qos.best_effort();
    pub_qos.durability_volatile();

    // Subscriptions from FMU: best effort, transient local
    rclcpp::QoS sub_qos(10);
    sub_qos.best_effort();
    sub_qos.transient_local();

    // Publishers
    offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", pub_qos);
    trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", pub_qos);
    vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", pub_qos);

    // Subscribers (supporting both versioned and unversioned topic names)
    local_pos_sub_v1_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", sub_qos,
      std::bind(&AltitudeControlNode::local_position_callback, this, std::placeholders::_1));

    local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", sub_qos,
      std::bind(&AltitudeControlNode::local_position_callback, this, std::placeholders::_1));

    // Control loop timer
    timer_ = this->create_wall_timer(
      timer_period_from_rate(publish_rate_hz_),
      std::bind(&AltitudeControlNode::control_timer_callback, this));
  }

  static bool is_telemetry_valid(
    const px4_msgs::msg::VehicleLocalPosition & msg,
    bool require_heading_good_for_control = false)
  {
    if (!msg.xy_valid || !msg.z_valid) {
      return false;
    }
    if (require_heading_good_for_control && !msg.heading_good_for_control) {
      return false;
    }
    if (!std::isfinite(msg.x) || !std::isfinite(msg.y) || !std::isfinite(msg.z) ||
      !std::isfinite(msg.heading))
    {
      return false;
    }
    return true;
  }

  bool is_telemetry_fresh() const
  {
    if (!has_local_position_) {
      return false;
    }
    auto now = std::chrono::steady_clock::now();
    return (now - last_position_rx_time_) <= TELEMETRY_FRESHNESS_TIMEOUT;
  }

  bool is_position_locked() const
  {
    return position_locked_;
  }

  bool has_local_position() const
  {
    return has_local_position_;
  }

  bool has_taken_off() const
  {
    return has_taken_off_;
  }

  bool is_heading_good_established() const
  {
    return heading_good_established_;
  }

  double hold_x() const
  {
    return hold_x_;
  }

  double hold_y() const
  {
    return hold_y_;
  }

  double hold_yaw() const
  {
    return hold_yaw_;
  }

  double current_x() const
  {
    return current_x_;
  }

  double current_y() const
  {
    return current_y_;
  }

  double current_z() const
  {
    return current_z_;
  }

  double current_yaw() const
  {
    return current_yaw_;
  }

private:
  static bool is_publish_rate_valid(double rate_hz)
  {
    if (!std::isfinite(rate_hz) || rate_hz <= 0.0) {
      return false;
    }
    const long double period_ns = 1'000'000'000.0L / static_cast<long double>(rate_hz);
    return std::isfinite(period_ns) &&
           period_ns <= static_cast<long double>(std::numeric_limits<int64_t>::max());
  }

  static std::chrono::nanoseconds timer_period_from_rate(double rate_hz)
  {
    if (!is_publish_rate_valid(rate_hz)) {
      throw std::invalid_argument("publish_rate_hz must produce a finite, representable period");
    }
    const long double period_ns = 1'000'000'000.0L / static_cast<long double>(rate_hz);
    const auto whole_nanoseconds = static_cast<int64_t>(period_ns);
    return std::chrono::nanoseconds(std::max<int64_t>(1, whole_nanoseconds));
  }

  void apply_pending_publish_rate()
  {
    const double requested_rate = pending_publish_rate_hz_.exchange(0.0, std::memory_order_acq_rel);
    if (requested_rate <= 0.0 || requested_rate == publish_rate_hz_) {
      return;
    }

    const auto new_period = timer_period_from_rate(requested_rate);
    auto previous_timer = timer_;
    if (previous_timer) {
      previous_timer->cancel();
    }
    timer_ = this->create_wall_timer(
      new_period, std::bind(&AltitudeControlNode::control_timer_callback, this));
    publish_rate_hz_ = requested_rate;
    RCLCPP_INFO(this->get_logger(), "Control timer now running at %.2f Hz", publish_rate_hz_);
  }

  void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    const bool require_heading_good = has_taken_off_ && heading_good_established_;
    if (!is_telemetry_valid(*msg, require_heading_good)) {
      auto now = std::chrono::steady_clock::now();
      if (now - last_invalid_log_time_ >= std::chrono::seconds(1)) {
        last_invalid_log_time_ = now;
        RCLCPP_WARN(this->get_logger(),
          "Rejecting invalid local position (require_heading_good=%d): "
          "xy_valid=%d, z_valid=%d, heading_good=%d, x=%.2f, y=%.2f, z=%.2f, heading=%.2f",
          require_heading_good, msg->xy_valid, msg->z_valid, msg->heading_good_for_control,
          msg->x, msg->y, msg->z, msg->heading);
      }
      return;
    }

    last_position_rx_time_ = std::chrono::steady_clock::now();
    current_x_ = msg->x;
    current_y_ = msg->y;
    current_z_ = msg->z;
    current_yaw_ = msg->heading;
    has_local_position_ = true;

    if (!position_locked_) {
      hold_x_ = current_x_;
      hold_y_ = current_y_;
      hold_yaw_ = current_yaw_;
      position_locked_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "Locked reference hold position: X=%.2f, Y=%.2f, Yaw=%.2f rad",
                  hold_x_, hold_y_, hold_yaw_);
    }

    const double current_alt = -current_z_;
    if (current_alt > 0.5) {
      has_taken_off_ = true;
    }
    if (has_taken_off_ && msg->heading_good_for_control && !heading_good_established_) {
      heading_good_established_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "In-flight heading alignment complete: heading_good_for_control=true "
                  "(alt=%.2f m, yaw=%.2f rad)",
                  current_alt, current_yaw_);
    }
  }

  void control_timer_callback()
  {
    apply_pending_publish_rate();

    // Readiness check 1: reference position must be locked from valid telemetry
    if (!position_locked_) {
      auto now = std::chrono::steady_clock::now();
      if (now - last_waiting_log_time_ >= std::chrono::seconds(1)) {
        last_waiting_log_time_ = now;
        RCLCPP_INFO(this->get_logger(),
                    "Waiting for valid local position telemetry to lock reference...");
      }
      return;
    }

    // Readiness check 2: before takeoff / control starts, telemetry must be fresh
    if (!has_taken_off_ && !is_telemetry_fresh()) {
      offboard_warmup_started_.reset();
      last_startup_command_time_.reset();
      auto now = std::chrono::steady_clock::now();
      if (now - last_waiting_log_time_ >= std::chrono::seconds(1)) {
        last_waiting_log_time_ = now;
        RCLCPP_WARN(this->get_logger(),
                    "Telemetry is stale (>%ld ms) before takeoff. Refusing to arm/control.",
                    TELEMETRY_FRESHNESS_TIMEOUT.count());
      }
      return;
    }

    const auto control_cycle_time = std::chrono::steady_clock::now();
    if (!has_taken_off_ && !offboard_warmup_started_) {
      offboard_warmup_started_ = control_cycle_time;
    }

    // 1. OffboardControlMode and TrajectorySetpoint must be published continuously
    publish_offboard_control_mode();
    publish_trajectory_setpoint();

    // 2. Offboard transition & Arming sequence. Keep the previous intended
    //    one-second warmup independent of the configured timer frequency.
    double current_alt = -current_z_;  // In NED: -z is altitude in meters
    if (current_alt > 0.5) {
      has_taken_off_ = true;
    }
    const auto now = std::chrono::steady_clock::now();
    if (!has_taken_off_ && offboard_warmup_started_ && current_alt < 0.3 &&
      now - *offboard_warmup_started_ >= OFFBOARD_WARMUP_DURATION &&
      (!last_startup_command_time_ ||
      now - *last_startup_command_time_ >= STARTUP_COMMAND_INTERVAL))
    {
      RCLCPP_INFO(this->get_logger(), "Requesting Offboard mode and Arming...");
      set_offboard_mode();
      arm();
      last_startup_command_time_ = now;
    }

    // 3. Periodic telemetry status log every ~1 second
    if (now - last_status_log_time_ >= std::chrono::seconds(1)) {
      last_status_log_time_ = now;
      RCLCPP_INFO(this->get_logger(),
                  "Current Altitude: %5.2f m | Target: %5.2f m | Hold: (X=%.2f, Y=%.2f)",
                  current_alt, target_altitude_m_, hold_x_, hold_y_);
    }
  }

  void publish_offboard_control_mode()
  {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;

    offboard_control_mode_pub_->publish(msg);
  }

  void publish_trajectory_setpoint()
  {
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    // NED frame:
    // x = north, y = east, z = down (negative altitude)
    msg.position[0] = static_cast<float>(hold_x_);
    msg.position[1] = static_cast<float>(hold_y_);
    msg.position[2] = static_cast<float>(-target_altitude_m_);

    // Maintain initial heading
    msg.yaw = static_cast<float>(hold_yaw_);

    trajectory_setpoint_pub_->publish(msg);
  }

  void set_offboard_mode()
  {
    // PX4 MAVLink DO_SET_MODE: param1 = 1 (custom mode), param2 = 6 (OFFBOARD)
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
  }

  void arm()
  {
    publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
      static_cast<float>(px4_msgs::msg::VehicleCommand::ARMING_ACTION_ARM));
  }

  void publish_vehicle_command(uint32_t command, float param1 = 0.0f, float param2 = 0.0f)
  {
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;

    vehicle_command_pub_->publish(msg);
  }

  // Node parameters
  double target_altitude_m_{2.0};
  double publish_rate_hz_{10.0};
  std::atomic<double> pending_publish_rate_hz_{0.0};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Reference hold positions
  bool position_locked_{false};
  bool has_local_position_{false};
  bool has_taken_off_{false};
  bool heading_good_established_{false};

  double hold_x_{0.0};
  double hold_y_{0.0};
  double hold_yaw_{0.0};

  // Current measurements
  double current_x_{0.0};
  double current_y_{0.0};
  double current_z_{0.0};
  double current_yaw_{0.0};

  static constexpr std::chrono::seconds OFFBOARD_WARMUP_DURATION{1};
  static constexpr std::chrono::seconds STARTUP_COMMAND_INTERVAL{1};
  std::optional<std::chrono::steady_clock::time_point> offboard_warmup_started_;
  std::optional<std::chrono::steady_clock::time_point> last_startup_command_time_;
  static constexpr std::chrono::milliseconds TELEMETRY_FRESHNESS_TIMEOUT{1000};
  std::chrono::steady_clock::time_point last_position_rx_time_{};
  std::chrono::steady_clock::time_point last_waiting_log_time_{};
  std::chrono::steady_clock::time_point last_invalid_log_time_{};
  std::chrono::steady_clock::time_point last_status_log_time_{};

  // ROS 2 Interfaces
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_v1_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

#ifndef ALTITUDE_CONTROL_TESTING
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<AltitudeControlNode>();
    rclcpp::spin(node);
  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(rclcpp::get_logger("altitude_control"), "Initialization error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
#endif
