#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from kvoy_msgs.msg import ImuData


def normalize3(v):
    norm = math.sqrt(sum(x * x for x in v))
    if norm < 1.0e-9:
        return (0.0, 0.0, 0.0)
    return tuple(x / norm for x in v)


def quat_normalize(q):
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def quat_to_euler_deg(q):
    x, y, z, w = quat_normalize(q)

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return tuple(math.degrees(v) for v in (roll, pitch, yaw))


def quat_rotate_inverse(q, v):
    x, y, z, w = quat_normalize(q)
    tx = 2.0 * (y * v[2] - z * v[1])
    ty = 2.0 * (z * v[0] - x * v[2])
    tz = 2.0 * (x * v[1] - y * v[0])
    return (
        v[0] - w * tx + (y * tz - z * ty),
        v[1] - w * ty + (z * tx - x * tz),
        v[2] - w * tz + (x * ty - y * tx),
    )


def mat3_mul_vec3(m, v):
    return (
        m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
        m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
        m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
    )


def dist3(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


class ImuDiagnostics(Node):
    def __init__(self):
        super().__init__("imu_diagnostics")
        self.declare_parameter(
            "imu_to_body_rotation",
            [1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0],
        )
        matrix = list(self.get_parameter("imu_to_body_rotation").value)
        if len(matrix) != 9:
            self.get_logger().error(
                "imu_to_body_rotation must contain 9 values; using identity")
            matrix = [1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0]
        self.imu_to_body_rotation = tuple(float(v) for v in matrix)
        self.create_subscription(ImuData, "/imu/data", self.on_imu, 10)
        self.last_print_time = self.get_clock().now()

    def on_imu(self, msg):
        now = self.get_clock().now()
        if (now - self.last_print_time).nanoseconds < 500_000_000:
            return
        self.last_print_time = now

        q = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        gyro = (
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        )
        accel = (
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        )
        accel_body = mat3_mul_vec3(self.imu_to_body_rotation, accel)
        accel_body_dir = normalize3(accel_body)
        expected_gravity_body = tuple(-x for x in accel_body_dir)
        gravity_sensor = quat_rotate_inverse(q, (0.0, 0.0, -1.0))
        gravity_body = mat3_mul_vec3(self.imu_to_body_rotation, gravity_sensor)
        gyro_body = mat3_mul_vec3(self.imu_to_body_rotation, gyro)
        roll, pitch, yaw = quat_to_euler_deg(q)

        self.get_logger().info(
            "sensor_euler_deg=[%.2f %.2f %.2f] gyro_body=[%.3f %.3f %.3f] "
            "gravity_body=[%.3f %.3f %.3f] -accel_body_dir=[%.3f %.3f %.3f] err=%.4f"
            % (
                roll,
                pitch,
                yaw,
                gyro_body[0],
                gyro_body[1],
                gyro_body[2],
                gravity_body[0],
                gravity_body[1],
                gravity_body[2],
                expected_gravity_body[0],
                expected_gravity_body[1],
                expected_gravity_body[2],
                dist3(gravity_body, expected_gravity_body),
            )
        )


def main():
    rclpy.init()
    node = ImuDiagnostics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
