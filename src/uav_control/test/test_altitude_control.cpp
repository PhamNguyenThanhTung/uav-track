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
 * Unit tests for AltitudeControlNode pre-takeoff and post-takeoff validity
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#define ALTITUDE_CONTROL_TESTING
#include "../src/altitude_control.cpp"  // NOLINT(build/include)

namespace
{

px4_msgs::msg::VehicleLocalPosition make_valid_ground_msg()
{
  px4_msgs::msg::VehicleLocalPosition msg{};
  msg.xy_valid = true;
  msg.z_valid = true;
  msg.heading_good_for_control = false;
  msg.x = 1.25f;
  msg.y = -0.75f;
  msg.z = -0.05f;
  msg.heading = 1.67f;
  return msg;
}

class AltitudeControlTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void publish_and_spin(
    const std::shared_ptr<AltitudeControlNode> & control_node,
    const px4_msgs::msg::VehicleLocalPosition & msg)
  {
    auto helper_node = std::make_shared<rclcpp::Node>("test_local_pos_publisher");
    rclcpp::QoS qos(10);
    qos.best_effort();
    qos.transient_local();
    auto pub = helper_node->create_publisher<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", qos);

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(control_node);
    exec.add_node(helper_node);

    pub->publish(msg);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
    while (std::chrono::steady_clock::now() < deadline) {
      exec.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
};

// 1. Ground + valid finite heading + heading_good_for_control=false -> accepted pre-takeoff
TEST_F(AltitudeControlTest, GroundFiniteHeadingWithHeadingGoodFalseAcceptedPreTakeoff)
{
  auto msg = make_valid_ground_msg();
  EXPECT_FALSE(msg.heading_good_for_control);
  EXPECT_TRUE(AltitudeControlNode::is_telemetry_valid(msg, false));
}

// 2. Finite heading with heading_good_for_control=false must not block initial reference lock
TEST_F(AltitudeControlTest, GroundHeadingGoodFalseDoesNotBlockReferenceLock)
{
  auto node = std::make_shared<AltitudeControlNode>();
  EXPECT_FALSE(node->is_position_locked());

  auto msg = make_valid_ground_msg();
  publish_and_spin(node, msg);

  EXPECT_TRUE(node->has_local_position());
  EXPECT_TRUE(node->is_position_locked());
  EXPECT_TRUE(node->is_telemetry_fresh());
  EXPECT_NEAR(node->hold_x(), 1.25, 1e-5);
  EXPECT_NEAR(node->hold_y(), -0.75, 1e-5);
  EXPECT_NEAR(node->hold_yaw(), 1.67, 1e-5);
}

// 3. NaN / Inf heading -> rejected and does not lock reference
TEST_F(AltitudeControlTest, NonFiniteHeadingRejectedAndDoesNotLockReference)
{
  auto nan_msg = make_valid_ground_msg();
  nan_msg.heading = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(AltitudeControlNode::is_telemetry_valid(nan_msg, false));

  auto inf_msg = make_valid_ground_msg();
  inf_msg.heading = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(AltitudeControlNode::is_telemetry_valid(inf_msg, false));

  auto node = std::make_shared<AltitudeControlNode>();
  publish_and_spin(node, nan_msg);
  EXPECT_FALSE(node->has_local_position());
  EXPECT_FALSE(node->is_position_locked());

  publish_and_spin(node, inf_msg);
  EXPECT_FALSE(node->has_local_position());
  EXPECT_FALSE(node->is_position_locked());
}

// 4. Invalid XY -> rejected and does not lock reference
TEST_F(AltitudeControlTest, InvalidXYRejected)
{
  auto flag_invalid = make_valid_ground_msg();
  flag_invalid.xy_valid = false;
  EXPECT_FALSE(AltitudeControlNode::is_telemetry_valid(flag_invalid, false));

  auto x_nan = make_valid_ground_msg();
  x_nan.x = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(AltitudeControlNode::is_telemetry_valid(x_nan, false));

  auto y_inf = make_valid_ground_msg();
  y_inf.y = -std::numeric_limits<float>::infinity();
  EXPECT_FALSE(AltitudeControlNode::is_telemetry_valid(y_inf, false));

  auto node = std::make_shared<AltitudeControlNode>();
  publish_and_spin(node, flag_invalid);
  EXPECT_FALSE(node->is_position_locked());
}

// 5. Invalid Z -> rejected and does not lock reference
TEST_F(AltitudeControlTest, InvalidZRejected)
{
  auto flag_invalid = make_valid_ground_msg();
  flag_invalid.z_valid = false;
  EXPECT_FALSE(AltitudeControlNode::is_telemetry_valid(flag_invalid, false));

  auto z_nan = make_valid_ground_msg();
  z_nan.z = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(AltitudeControlNode::is_telemetry_valid(z_nan, false));

  auto node = std::make_shared<AltitudeControlNode>();
  publish_and_spin(node, flag_invalid);
  EXPECT_FALSE(node->is_position_locked());
}

// 6. Stale telemetry -> rejected before takeoff
TEST_F(AltitudeControlTest, StaleTelemetryRejectedBeforeTakeoff)
{
  auto node = std::make_shared<AltitudeControlNode>();
  EXPECT_FALSE(node->is_telemetry_fresh());

  auto msg = make_valid_ground_msg();
  publish_and_spin(node, msg);
  EXPECT_TRUE(node->is_position_locked());
  EXPECT_TRUE(node->is_telemetry_fresh());

  // Wait beyond TELEMETRY_FRESHNESS_TIMEOUT (1000 ms) without new messages
  std::this_thread::sleep_for(std::chrono::milliseconds(1100));
  EXPECT_FALSE(node->is_telemetry_fresh());
}

// 7. Post-takeoff heading-invalid behavior -> once airborne and heading_good_for_control
//    has been established, subsequent messages with heading_good_for_control=false or
//    non-finite heading are rejected.
TEST_F(AltitudeControlTest, PostTakeoffHeadingInvalidRejectedAfterInFlightAlignment)
{
  auto msg = make_valid_ground_msg();
  // Direct static policy check when require_heading_good_for_control=true
  EXPECT_FALSE(AltitudeControlNode::is_telemetry_valid(msg, true));
  msg.heading_good_for_control = true;
  EXPECT_TRUE(AltitudeControlNode::is_telemetry_valid(msg, true));

  // Node lifecycle check: ground -> climb (<1.5 m) -> in-flight alignment (>1.5 m) -> drop
  auto node = std::make_shared<AltitudeControlNode>();

  // Step A: Ground reference lock with heading_good_for_control=false
  auto ground_msg = make_valid_ground_msg();
  publish_and_spin(node, ground_msg);
  EXPECT_TRUE(node->is_position_locked());
  EXPECT_FALSE(node->has_taken_off());
  EXPECT_FALSE(node->is_heading_good_established());

  // Step B: Initial climb above 0.5 m before PX4 1.5 m mag alignment completes
  auto climb_msg = make_valid_ground_msg();
  climb_msg.z = -1.0f;
  climb_msg.heading_good_for_control = false;
  publish_and_spin(node, climb_msg);
  EXPECT_TRUE(node->has_taken_off());
  EXPECT_FALSE(node->is_heading_good_established());
  EXPECT_NEAR(node->current_z(), -1.0, 1e-5);

  // Step C: Climb above 1.5 m where PX4 sets heading_good_for_control=true
  auto aligned_msg = make_valid_ground_msg();
  aligned_msg.z = -1.8f;
  aligned_msg.heading_good_for_control = true;
  publish_and_spin(node, aligned_msg);
  EXPECT_TRUE(node->has_taken_off());
  EXPECT_TRUE(node->is_heading_good_established());
  EXPECT_NEAR(node->current_z(), -1.8, 1e-5);

  // Step D: Post-takeoff message with heading_good_for_control=false is rejected
  auto degraded_msg = make_valid_ground_msg();
  degraded_msg.z = -2.4f;
  degraded_msg.heading_good_for_control = false;
  publish_and_spin(node, degraded_msg);
  EXPECT_NEAR(node->current_z(), -1.8, 1e-5);
  EXPECT_NEAR(node->hold_yaw(), 1.67, 1e-5);
}

}  // namespace
