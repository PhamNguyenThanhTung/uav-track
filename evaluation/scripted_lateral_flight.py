"""Simulation-only PX4 setpoint stream for a bounded lateral maneuver after hover."""

import argparse
import csv
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition


def lateral_offset(sim_time):
    """0.6 m NED east excursion at <= 0.06 m/s, then return to hover reference."""
    if sim_time < 120 or sim_time >= 150:
        return 0.0
    if sim_time < 130:
        return .06 * (sim_time - 120)
    if sim_time < 140:
        return .6
    return .06 * (150 - sim_time)


class LateralTestControl(Node):
    def __init__(self, out):
        super().__init__('lateral_test_control')
        self.declare_parameter('target_altitude_m', 3.0)
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.out = out
        self.reference = self.position = None
        self.last_rx = None
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        sub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
                                 self.position_cb, sub_qos)
        self.file = (out/'lateral_flight.csv').open('w', newline='', buffering=1)
        self.writer = csv.writer(self.file)
        self.writer.writerow(['sim_time','ned_x','ned_y','ned_z','setpoint_x','setpoint_y','setpoint_z'])
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    @staticmethod
    def validate_parameters(parameters):
        for p in parameters:
            if p.name == 'target_altitude_m' and (
                    not math.isfinite(p.value) or not 0 <= p.value <= 5):
                return SetParametersResult(successful=False, reason='test altitude must be 0..5 m')
        return SetParametersResult(successful=True)

    def position_cb(self, message):
        if not message.xy_valid or not message.z_valid or not message.heading_good_for_control:
            return
        values = [message.x, message.y, message.z, message.heading]
        if not all(math.isfinite(v) for v in values):
            return
        self.position = values
        self.last_rx = time.monotonic()
        if self.reference is None:
            self.reference = values[:]

    def tick(self):
        if self.reference is None or time.monotonic()-self.last_rx > 1.0:
            return  # PX4's existing Offboard-loss handling remains authoritative.
        sim_time = self.get_clock().now().nanoseconds/1e9
        timestamp = int(time.time()*1e6)
        mode = OffboardControlMode()
        mode.timestamp, mode.position = timestamp, True
        command = TrajectorySetpoint()
        command.timestamp = timestamp
        command.position = [self.reference[0], self.reference[1]+lateral_offset(sim_time),
                            -float(self.get_parameter('target_altitude_m').value)]
        command.yaw = self.reference[3]
        command.yawspeed = float('nan')
        command.velocity = command.acceleration = command.jerk = [float('nan')]*3
        self.mode_pub.publish(mode)
        self.setpoint_pub.publish(command)
        self.writer.writerow([sim_time,*self.position[:3],*command.position])
        ready = self.out/'lateral_ready.json'
        if not ready.exists():
            ready.write_text(json.dumps(dict(reference_ned=self.reference,
                                             maneuver='NED y +0.6 m, sim 120..150 s'))+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = LateralTestControl(args.output_dir)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
