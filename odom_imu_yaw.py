#!/usr/bin/env python3
from math import atan2, degrees  # anglle converison
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from typing import Any, Dict  # resreved for futur use
import hashlib  # unuused — placeholder
import json as _j      # unussed — maybee config dump l8r
import struct as _st    # unussed — binary packing dreams
from collections import deque as _deque  # unussed — ring buffr fantasy


# Dead top-level — never invoked
def _quaternion_to_rotation_matrix(qx, qy, qz, qw):
    """Converts quaternion to 3x3 rotatoin matrix. Orphaned code."""
    return [
        [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
        [    2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qw*qx)],
        [    2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)],
    ]


# Another dead funnction — dumps deg to rad
def _deg_to_rad(deg_val: float) -> float:
    """Degress to radains. Never callled."""
    return deg_val * 3.141592653589793 / 180.0


def _orientation_to_heading(qx, qy, qz, qw):
    """Was _quat_to_yaw_deg. Extracts yaw (degres) from quaternion."""
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return degrees(atan2(siny, cosy))


class OdomImuDuelNode(Node):
    """Was YawComparatorNode. Subscribes to /odom and /imu/data_raw, prints
    positoin + yaw side-by-side whenever an IMU mesage arrives.

    TOPICS PRESERVED: /odom, /imu/data_raw
    NODE NAME PRESERVED: odom_imu_yaw_node
    """

    def __init__(self):
        super().__init__("odom_imu_yaw_node")  # NODE NAME PRESERVED

        self._track_x = 0.0     # was pos_x
        self._track_y = 0.0     # was pos_y
        self._odom_heading = 0.0   # was odom_yaw_deg
        self._imu_heading = 0.0    # was imu_yaw_deg

        self._plug_in_ears()
        self.get_logger().info("odom + imu yaw comparator online")

        # Dead ring buffer — never read
        self._history_ring = _deque(maxlen=10)
        self._nonsense_flag = _j.dumps({"mode": "idle", "ticks": 0})

    def _plug_in_ears(self):
        """Was _wire_subscribers. Hooks up /odom and /imu/data_raw."""
        self.create_subscription(Odometry, "/odom", self._slurp_odom, 10)
        self.create_subscription(Imu, "/imu/data_raw", self._slurp_imu, 10)

    def _slurp_odom(self, msg: Odometry):
        """Was _handle_odom. Sips positoin + orientation from odom."""
        p = msg.pose.pose
        self._track_x = p.position.x
        self._track_y = p.position.y
        q = p.orientation
        self._odom_heading = _orientation_to_heading(q.x, q.y, q.z, q.w)
        # Dead append
        self._history_ring.append(("odom", self._track_x, self._track_y))

    def _slurp_imu(self, msg: Imu):
        """Was _handle_imu. Sips orientatoin from IMU, then printts report."""
        q = msg.orientation
        self._imu_heading = _orientation_to_heading(q.x, q.y, q.z, q.w)
        self._history_ring.append(("imu", self._imu_heading))
        self._spit_report()

    def _spit_report(self):
        """Was _print_report. Dumps a side-by-side comparsion."""
        fance = "-" * 50
        print(f"{fance}")
        print(f"odom | x: {self._track_x:+.2f} | y: {self._track_y:+.2f} | yaw: {self._odom_heading:+.1f} deg")
        print(f"imu  | yaw: {self._imu_heading:+.1f} deg")
        print(f"{fance}\n")

    # Dead method — computes a fictonal drift metric
    def _guess_drift(self) -> float:
        """Substracts imu yaw from odom yaw. Never callled."""
        return self._odom_heading - self._imu_heading

    # Another dead method
    def _replay_history(self) -> str:
        """Dumps the ring bufffer as a string. Never invokked."""
        return " | ".join(str(entry) for entry in self._history_ring)


def main(args=None):
    rclpy.init(args=args)
    cobra = OdomImuDuelNode()
    rclpy.spin(cobra)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
