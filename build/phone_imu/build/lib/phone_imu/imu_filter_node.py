#!/usr/bin/env python3
"""
imu_filter_node.py
------------------
Complementary filter for IMU data.

Subscribes : /imu/data_raw  (sensor_msgs/Imu)
Publishes  : /imu/data      (sensor_msgs/Imu) — with orientation quaternion

Algorithm:
  roll/pitch from accelerometer  (accurate long-term, noisy short-term)
  roll/pitch from gyroscope      (accurate short-term, drifts long-term)

  angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel_angle

  alpha = 0.98  (98% trust gyro, 2% correct from accel)

Yaw is integrated from gyro only (no magnetometer = yaw drifts).
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import tf2_ros


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert roll/pitch/yaw (radians) to a geometry_msgs/Quaternion."""
    cr = math.cos(roll  * 0.5)
    sr = math.sin(roll  * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw   * 0.5)
    sy = math.sin(yaw   * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class ImuFilterNode(Node):
    """Complementary filter: fuses gyro + accelerometer into an orientation."""

    def __init__(self):
        super().__init__('imu_filter_node')

        # alpha: weight given to gyroscope integration (0.0 – 1.0)
        self.declare_parameter('alpha', 0.98)
        self.alpha = self.get_parameter('alpha').get_parameter_value().double_value

        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self._imu_callback,
            10
        )
        self.publisher_  = self.create_publisher(Imu,         '/imu/data',    10)
        self.marker_pub  = self.create_publisher(MarkerArray, '/imu/markers', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw   = 0.0
        self._last_time = None

        self.get_logger().info(
            f"ImuFilterNode started — alpha={self.alpha:.2f} "
            f"(gyro weight), publishing on /imu/data and /imu/markers"
        )

    def _imu_callback(self, msg: Imu):
        now = self.get_clock().now()

        # ── Compute dt ──────────────────────────────────────────────────────
        if self._last_time is None:
            self._last_time = now
            return
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now

        if dt <= 0.0 or dt > 1.0:
            # Skip bad dt (e.g. first callback or long pause)
            return

        # ── Raw sensor values ────────────────────────────────────────────────
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        gx = msg.angular_velocity.x   # rad/s
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        # ── Accelerometer-derived tilt (reliable long-term) ──────────────────
        accel_roll  = math.atan2(ay, math.sqrt(ax * ax + az * az))
        accel_pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        # ── Gyro integration (reliable short-term) ───────────────────────────
        gyro_roll  = self.roll  + gx * dt
        gyro_pitch = self.pitch + gy * dt
        gyro_yaw   = self.yaw   + gz * dt   # yaw has no accel correction

        # ── Complementary fusion ─────────────────────────────────────────────
        self.roll  = self.alpha * gyro_roll  + (1.0 - self.alpha) * accel_roll
        self.pitch = self.alpha * gyro_pitch + (1.0 - self.alpha) * accel_pitch
        self.yaw   = gyro_yaw  # gyro-only for yaw

        # ── Build output message ─────────────────────────────────────────────
        out = Imu()
        out.header.stamp    = msg.header.stamp
        out.header.frame_id = "phone_imu"

        out.orientation = euler_to_quaternion(self.roll, self.pitch, self.yaw)
        out.orientation_covariance[0] = 0.01  # known orientation

        out.angular_velocity   = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration

        out.angular_velocity_covariance[0]    = msg.angular_velocity_covariance[0]
        out.linear_acceleration_covariance[0] = msg.linear_acceleration_covariance[0]

        self.publisher_.publish(out)

        # ── Broadcast TF: world → phone_imu (applies the orientation) ────────
        tf_msg = TransformStamped()
        tf_msg.header.stamp    = msg.header.stamp
        tf_msg.header.frame_id = 'world'
        tf_msg.child_frame_id  = 'phone_imu'
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation      = out.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

        self._publish_markers(out)



    def _publish_markers(self, imu_msg: Imu):
        """Publish RViz2 markers in the phone_imu frame."""
        from geometry_msgs.msg import Point
        markers = MarkerArray()
        stamp = imu_msg.header.stamp
        frame = 'phone_imu'

        ax = imu_msg.linear_acceleration.x
        ay = imu_msg.linear_acceleration.y
        az = imu_msg.linear_acceleration.z
        gx = imu_msg.angular_velocity.x
        gy = imu_msg.angular_velocity.y
        gz = imu_msg.angular_velocity.z
        gyro_mag = math.sqrt(gx*gx + gy*gy + gz*gz)

        t = min(gyro_mag / 3.0, 1.0)
        r_col = float(t)
        b_col = float(1.0 - t)

        # ── 1. Acceleration arrow ─────────────────────────────────────────────
        acc_arrow = Marker()
        acc_arrow.header.stamp    = stamp
        acc_arrow.header.frame_id = frame
        acc_arrow.ns     = 'phone_imu'
        acc_arrow.id     = 1
        acc_arrow.type   = Marker.ARROW
        acc_arrow.action = Marker.ADD
        acc_arrow.pose.orientation.w = 1.0
        acc_arrow.scale.x = 0.03
        acc_arrow.scale.y = 0.06
        acc_arrow.scale.z = 0.0
        p0 = Point(); p0.x = 0.0; p0.y = 0.0; p0.z = 0.0
        p1 = Point(); p1.x = ax * 0.05; p1.y = ay * 0.05; p1.z = az * 0.05
        acc_arrow.points = [p0, p1]
        acc_arrow.color.r = 0.1; acc_arrow.color.g = 1.0
        acc_arrow.color.b = 0.3; acc_arrow.color.a = 1.0
        markers.markers.append(acc_arrow)

        # ── 3. Angular velocity arrow ─────────────────────────────────────────
        gyro_arrow = Marker()
        gyro_arrow.header.stamp    = stamp
        gyro_arrow.header.frame_id = frame
        gyro_arrow.ns     = 'phone_imu'
        gyro_arrow.id     = 2
        gyro_arrow.type   = Marker.ARROW
        gyro_arrow.action = Marker.ADD
        gyro_arrow.pose.orientation.w = 1.0
        gyro_arrow.scale.x = 0.025
        gyro_arrow.scale.y = 0.05
        gyro_arrow.scale.z = 0.0
        gp0 = Point(); gp0.x = 0.0; gp0.y = 0.0; gp0.z = 0.0
        gp1 = Point(); gp1.x = gx * 0.2; gp1.y = gy * 0.2; gp1.z = gz * 0.2
        gyro_arrow.points = [gp0, gp1]
        gyro_arrow.color.r = 0.1; gyro_arrow.color.g = 0.9
        gyro_arrow.color.b = 1.0; gyro_arrow.color.a = 1.0
        markers.markers.append(gyro_arrow)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = ImuFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
