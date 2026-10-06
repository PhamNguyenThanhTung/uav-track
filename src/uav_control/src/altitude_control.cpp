/****************************************************************************
 *
 * Copyright (c) 2026 UAV Project.
 * All rights reserved.
 *
 * ROS 2 Altitude Controller for UAV using PX4 Offboard Mode
 *
 ****************************************************************************/

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
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
  AltitudeControlNode() : Node("altitude_control")
  {
    // Declare parameters with default values
    this->declare_parameter<double>("target_altitude_m", 2.0);
    this->declare_parameter<double>("publish_rate_hz", 10.0);

    target_altitude_m_ = this->get_parameter("target_altitude_m").as_double();
    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();

    RCLCPP_INFO(this->get_logger(), "Starting AltitudeControlNode");
    RCLCPP_INFO(this->get_logger(), "Target altitude: %.2f m", target_altitude_m_);
    RCLCPP_INFO(this->get_logger(), "Publish rate: %.1f Hz", publish_rate_hz_);

    // Register parameter callback for dynamic runtime updates
    param_callback_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &param : parameters) {
          if (param.get_name() == "target_altitude_m") {
            target_altitude_m_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Target altitude updated to: %.2f m", target_altitude_m_);
          } else if (param.get_name() == "publish_rate_hz") {
            double new_rate = param.as_double();
            if (new_rate > 0.0) {
              publish_rate_hz_ = new_rate;
              RCLCPP_INFO(this->get_logger(), "Publish rate updated to: %.1f Hz", publish_rate_hz_);
            } else {
              result.successful = false;
              result.reason = "publish_rate_hz must be > 0";
            }
          }
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
    auto timer_period = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / publish_rate_hz_));
    timer_ = this->create_wall_timer(
      timer_period, std::bind(&AltitudeControlNode::control_timer_callback, this));
  }

private:
  void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    current_x_ = msg->x;
    current_y_ = msg->y;
    current_z_ = msg->z;
    current_yaw_ = msg->heading;

    if (!position_locked_) {
      hold_x_ = current_x_;
      hold_y_ = current_y_;
      hold_yaw_ = current_yaw_;
      position_locked_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "Locked reference hold position: X=%.2f, Y=%.2f, Yaw=%.2f rad",
                  hold_x_, hold_y_, hold_yaw_);
    }
    has_local_position_ = true;
  }

  void control_timer_callback()
  {
    // 1. OffboardControlMode and TrajectorySetpoint must be published continuously
    publish_offboard_control_mode();
    publish_trajectory_setpoint();

    // 2. Offboard transition & Arming sequence
    // Continuously command Offboard mode and Arming every 10 cycles (~1s) until drone has lifted off
    double current_alt = -current_z_; // In NED: -z is altitude in meters
    if (current_alt > 0.5) {
      has_taken_off_ = true;
    }
    if (!has_taken_off_ && cycle_count_ >= 10 && cycle_count_ % 10 == 0) {
      if (current_alt < 0.3) {
        RCLCPP_INFO(this->get_logger(), "Requesting Offboard mode and Arming...");
        set_offboard_mode();
        arm();
      }
    }

    // 3. Periodic telemetry status log every ~1 second
    if (cycle_count_ % static_cast<uint64_t>(publish_rate_hz_) == 0) {
      RCLCPP_INFO(this->get_logger(),
                  "Current Altitude: %5.2f m | Target: %5.2f m | Hold: (X=%.2f, Y=%.2f)",
                  current_alt, target_altitude_m_, hold_x_, hold_y_);
    }

    cycle_count_++;
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
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Reference hold positions
  bool position_locked_{false};
  bool has_local_position_{false};
  bool has_taken_off_{false};

  double hold_x_{0.0};
  double hold_y_{0.0};
  double hold_yaw_{0.0};

  // Current measurements
  double current_x_{0.0};
  double current_y_{0.0};
  double current_z_{0.0};
  double current_yaw_{0.0};

  uint64_t cycle_count_{0};

  // ROS 2 Interfaces
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_v1_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AltitudeControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
