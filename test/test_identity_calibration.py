#!/usr/bin/env python3

import math
import time
import unittest

import launch
import launch_testing
from launch_ros.actions import Node
import rclpy
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def generate_test_description():
    calibrator = Node(
        package="multi_lidar_calibrator",
        executable="multi_lidar_calibrator",
        name="multi_lidar_calibrator",
        output="screen",
        parameters=[
            {
                "points_parent_src": "/test/points_parent",
                "points_child_src": "/test/points_child",
                "voxel_size": 0.5,
                "ndt_epsilon": 0.01,
                "ndt_step_size": 0.1,
                "ndt_resolution": 1.0,
                "ndt_iterations": 1,
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
            }
        ],
    )

    return (
        launch.LaunchDescription(
            [calibrator, launch_testing.actions.ReadyToTest()]
        ),
        {"calibrator": calibrator},
    )


class TestIdentityCalibration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_identity_calibration")
        cls.output = None
        cls.parent_pub = cls.node.create_publisher(
            PointCloud2, "/test/points_parent", qos_profile_sensor_data
        )
        cls.child_pub = cls.node.create_publisher(
            PointCloud2, "/test/points_child", qos_profile_sensor_data
        )
        cls.output_sub = cls.node.create_subscription(
            PointCloud2,
            "/points_calibrated",
            cls._output_callback,
            10,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _output_callback(cls, msg):
        cls.output = msg

    def test_identical_clouds_publish_identity_calibration(self):
        discovery_deadline = time.monotonic() + 10.0
        while time.monotonic() < discovery_deadline:
            if (
                self.parent_pub.get_subscription_count() > 0
                and self.child_pub.get_subscription_count() > 0
                and self.output_sub.get_publisher_count() > 0
            ):
                break
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.assertGreater(self.parent_pub.get_subscription_count(), 0)
        self.assertGreater(self.child_pub.get_subscription_count(), 0)
        self.assertGreater(self.output_sub.get_publisher_count(), 0)

        deadline = time.monotonic() + 30.0
        published_stamps = set()
        child_cloud = None
        while self.output is None and time.monotonic() < deadline:
            child_nanoseconds = self.node.get_clock().now().nanoseconds
            parent_nanoseconds = child_nanoseconds - 1_000_000
            parent_stamp = _time_message(
                parent_nanoseconds // 1_000_000_000,
                parent_nanoseconds % 1_000_000_000,
            )
            child_stamp = _time_message(
                child_nanoseconds // 1_000_000_000,
                child_nanoseconds % 1_000_000_000,
            )
            parent_cloud = self._make_cloud("parent_frame", parent_stamp)
            child_cloud = self._make_cloud("child_frame", child_stamp)
            published_stamps.add((child_stamp.sec, child_stamp.nanosec))
            self.parent_pub.publish(parent_cloud)
            self.child_pub.publish(child_cloud)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertIsNotNone(self.output, "Timed out waiting for calibrated cloud")
        self.assertEqual("parent_frame", self.output.header.frame_id)
        self.assertIn(
            (self.output.header.stamp.sec, self.output.header.stamp.nanosec),
            published_stamps,
        )

        output_points = list(
            point_cloud2.read_points(
                self.output,
                field_names=("x", "y", "z", "intensity"),
                skip_nans=False,
            )
        )
        expected_points = list(
            point_cloud2.read_points(
                child_cloud,
                field_names=("x", "y", "z", "intensity"),
                skip_nans=False,
            )
        )

        self.assertEqual(len(expected_points), len(output_points))
        for expected, actual in zip(expected_points, output_points):
            for field_name in ("x", "y", "z"):
                expected_value = expected[field_name]
                actual_value = actual[field_name]
                self.assertTrue(math.isfinite(actual_value))
                self.assertAlmostEqual(
                    expected_value,
                    actual_value,
                    delta=0.1,
                    msg="Expected identity transform, got {} vs {}".format(
                        expected, actual
                    ),
                )
            self.assertAlmostEqual(
                expected["intensity"], actual["intensity"], delta=1e-6
            )

    @staticmethod
    def _make_cloud(frame_id, stamp):
        header = Header(stamp=stamp, frame_id=frame_id)
        fields = [
            PointField(
                name="x", offset=0, datatype=PointField.FLOAT32, count=1
            ),
            PointField(
                name="y", offset=4, datatype=PointField.FLOAT32, count=1
            ),
            PointField(
                name="z", offset=8, datatype=PointField.FLOAT32, count=1
            ),
            PointField(
                name="intensity", offset=16, datatype=PointField.FLOAT32, count=1
            ),
        ]
        points = []
        offsets = (0.15, 0.30)
        for center_x in (0.0, 1.5, 3.0):
            for center_y in (0.0, 1.5, 3.0):
                for center_z in (0.0, 1.5):
                    for offset_x in offsets:
                        for offset_y in offsets:
                            for offset_z in offsets:
                                x = center_x + offset_x
                                y = center_y + offset_y
                                z = center_z + offset_z
                                intensity = 1.0 + x + y + z
                                points.append((x, y, z, intensity))
        return point_cloud2.create_cloud(header, fields, points)


def _time_message(sec, nanosec):
    return Time(sec=sec, nanosec=nanosec)


if __name__ == "__main__":
    unittest.main()
