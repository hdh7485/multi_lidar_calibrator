#!/usr/bin/env python3

import math
import random
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


TRANSLATION = (0.75, -0.40, 0.25)
YAW = 0.20


def generate_test_description():
    calibrator = Node(
        package="multi_lidar_calibrator",
        executable="multi_lidar_calibrator",
        name="known_transform_calibrator",
        output="screen",
        parameters=[
            {
                "points_parent_src": "/test_transform/points_parent",
                "points_child_src": "/test_transform/points_child",
                "voxel_size": 0.15,
                "ndt_epsilon": 0.001,
                "ndt_step_size": 0.1,
                "ndt_resolution": 0.5,
                "ndt_iterations": 60,
                "x": 0.65,
                "y": -0.32,
                "z": 0.20,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.16,
            }
        ],
    )

    return (
        launch.LaunchDescription(
            [calibrator, launch_testing.actions.ReadyToTest()]
        ),
        {"calibrator": calibrator},
    )


class TestKnownTransformCalibration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_known_transform_calibration")
        cls.output = None
        cls.parent_pub = cls.node.create_publisher(
            PointCloud2,
            "/test_transform/points_parent",
            qos_profile_sensor_data,
        )
        cls.child_pub = cls.node.create_publisher(
            PointCloud2,
            "/test_transform/points_child",
            qos_profile_sensor_data,
        )
        cls.output_sub = cls.node.create_subscription(
            PointCloud2, "/points_calibrated", cls._output_callback, 10
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _output_callback(cls, msg):
        cls.output = msg

    def test_recovers_known_translation_and_yaw(self):
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

        child_points = self._make_asymmetric_points()
        parent_points = [self._transform_point(point) for point in child_points]
        deadline = time.monotonic() + 30.0
        published_stamps = set()
        while self.output is None and time.monotonic() < deadline:
            now_ns = self.node.get_clock().now().nanoseconds
            child_stamp = _time_message(
                now_ns // 1_000_000_000, now_ns % 1_000_000_000
            )
            published_stamps.add((child_stamp.sec, child_stamp.nanosec))
            parent_stamp = _time_message(
                (now_ns - 1_000_000) // 1_000_000_000,
                (now_ns - 1_000_000) % 1_000_000_000,
            )
            self.parent_pub.publish(
                self._make_cloud("parent_frame", parent_stamp, parent_points)
            )
            self.child_pub.publish(
                self._make_cloud("child_frame", child_stamp, child_points)
            )
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
        self.assertEqual(len(parent_points), len(output_points))
        squared_position_errors = []
        for expected, actual in zip(parent_points, output_points):
            squared_position_errors.append(
                sum(
                    (expected[index] - actual[field_name]) ** 2
                    for index, field_name in enumerate(("x", "y", "z"))
                )
            )
            self.assertAlmostEqual(expected[3], actual["intensity"], delta=1e-6)

        position_rmse = math.sqrt(
            sum(squared_position_errors) / len(squared_position_errors)
        )
        self.assertLess(
            position_rmse,
            0.14,
            "Known transform RMSE exceeded 14 cm: {:.3f} m".format(
                position_rmse
            ),
        )

    @staticmethod
    def _make_asymmetric_points():
        randomizer = random.Random(7485)
        points = []
        clusters = (
            (-2.2, -1.1, 0.2),
            (-0.4, 2.0, 1.3),
            (1.7, -0.2, -0.8),
            (3.1, 1.4, 0.6),
        )
        for cluster_index, center in enumerate(clusters):
            for point_index in range(90):
                x = center[0] + randomizer.uniform(-0.45, 0.45)
                y = center[1] + randomizer.uniform(-0.30, 0.30)
                z = center[2] + randomizer.uniform(-0.20, 0.20)
                intensity = float(cluster_index * 100 + point_index)
                points.append((x, y, z, intensity))
        return points

    @staticmethod
    def _transform_point(point):
        cos_yaw = math.cos(YAW)
        sin_yaw = math.sin(YAW)
        x, y, z, intensity = point
        return (
            cos_yaw * x - sin_yaw * y + TRANSLATION[0],
            sin_yaw * x + cos_yaw * y + TRANSLATION[1],
            z + TRANSLATION[2],
            intensity,
        )

    @staticmethod
    def _make_cloud(frame_id, stamp, points):
        header = Header(stamp=stamp, frame_id=frame_id)
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity",
                offset=16,
                datatype=PointField.FLOAT32,
                count=1,
            ),
        ]
        return point_cloud2.create_cloud(header, fields, points)


def _time_message(sec, nanosec):
    return Time(sec=sec, nanosec=nanosec)


if __name__ == "__main__":
    unittest.main()
