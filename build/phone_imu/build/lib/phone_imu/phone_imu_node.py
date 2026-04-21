#!/usr/bin/env python3
"""
phone_imu_node.py
-----------------
Polls the Phyphox app's HTTP REST API and publishes raw IMU data
as sensor_msgs/Imu on /imu/data_raw at ~50 Hz.

Setup:
  1. Install Phyphox on your phone.
  2. Open "Accelerometer" and "Gyroscope" experiments.
  3. Enable Remote Access -> note the IP (e.g. 192.168.1.42).
  4. Set PHONE_IP below to that address.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import requests

# ── Change these to match what Phyphox Remote Access shows ─────────────────
PHONE_IP   = "10.254.62.108"   # try 192.0.0.3 if this doesn't work
PHONE_PORT = 8080
# ───────────────────────────────────────────────────────────────────────────


class PhoneIMUNode(Node):
    """Reads accelerometer + gyroscope data from Phyphox and publishes it."""

    def __init__(self):
        super().__init__('phone_imu_node')

        # Parameters overridable at launch:
        #   ros2 run phone_imu phone_imu_node --ros-args -p phone_ip:=<IP> -p phone_port:=8080
        self.declare_parameter('phone_ip',   PHONE_IP)
        self.declare_parameter('phone_port', PHONE_PORT)
        self.phone_ip   = self.get_parameter('phone_ip').get_parameter_value().string_value
        self.phone_port = self.get_parameter('phone_port').get_parameter_value().integer_value

        self.publisher_ = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz

        self._url = (
            f"http://{self.phone_ip}:{self.phone_port}/get?"
            "accX&accY&accZ&gyrX&gyrY&gyrZ&graX&graY&graZ"
        )

        self.get_logger().info(
            f"PhoneIMUNode started — polling http://{self.phone_ip}:{self.phone_port} at 50 Hz"
        )

    def _get_value(self, buf: dict, key: str, default: float = 0.0) -> float:
        """Safely extract the latest value from a Phyphox buffer dict."""
        try:
            val = buf[key]["buffer"][-1]
            if val is None:
                return default
            return float(val)
        except (KeyError, IndexError, TypeError):
            return default

    def timer_callback(self):
        try:
            r = requests.get(self._url, timeout=0.1)
            r.raise_for_status()
            buf = r.json()["buffer"]

            ax = self._get_value(buf, "accX")
            ay = self._get_value(buf, "accY")
            az = self._get_value(buf, "accZ")

            # Phyphox gyro channel name varies by experiment:
            #   "gyroX" → Gyroscope experiment
            #   "gyrX"  → Acceleration (with g) / Inertial Sensor experiment
            # Auto-detect which one is present.
            if "gyroX" in buf:
                gx_key, gy_key, gz_key = "gyroX", "gyroY", "gyroZ"
            elif "gyrX" in buf:
                gx_key, gy_key, gz_key = "gyrX", "gyrY", "gyrZ"
            else:
                gx_key, gy_key, gz_key = None, None, None

            has_gyro = gx_key is not None
            gx = self._get_value(buf, gx_key) if has_gyro else 0.0
            gy = self._get_value(buf, gy_key) if has_gyro else 0.0
            gz = self._get_value(buf, gz_key) if has_gyro else 0.0


            if not has_gyro:
                self.get_logger().warn(
                    "Gyroscope data not found in Phyphox response.\n"
                    "  → Open the 'Gyroscope with g' experiment in Phyphox\n"
                    "    (or any experiment that includes BOTH acc + gyro).\n"
                    "  Publishing accel-only until gyro is available.",
                    throttle_duration_sec=10.0
                )

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "phone_imu"

            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az

            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz

            # Covariances
            msg.orientation_covariance[0] = -1.0          # unknown
            msg.linear_acceleration_covariance[0] = 0.01
            msg.angular_velocity_covariance[0] = 0.01 if has_gyro else -1.0

            self.publisher_.publish(msg)

        except requests.exceptions.Timeout:
            self.get_logger().warn(
                "Timeout reaching phone — is Phyphox running with Remote Access enabled?",
                throttle_duration_sec=5.0
            )
        except Exception as e:
            self.get_logger().warn(
                f"Error reading IMU data: {e}",
                throttle_duration_sec=5.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = PhoneIMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
