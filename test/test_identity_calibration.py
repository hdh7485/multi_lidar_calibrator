#!/usr/bin/env python

import math
import threading
import time
import unittest

import rospy
import rostest
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as point_cloud2
from std_msgs.msg import Header


class IdentityCalibrationTest(unittest.TestCase):
    def setUp(self):
        rospy.init_node("test_identity_calibration", anonymous=True)
        self.output = None
        self.output_event = threading.Event()
        self.parent_pub = rospy.Publisher(
            "/test/points_parent", PointCloud2, queue_size=1
        )
        self.child_pub = rospy.Publisher(
            "/test/points_child", PointCloud2, queue_size=1
        )
        self.output_sub = rospy.Subscriber(
            "/points_calibrated", PointCloud2, self._output_callback
        )

    def _output_callback(self, msg):
        self.output = msg
        self.output_event.set()

    def test_identical_clouds_publish_identity_calibration(self):
        parent_stamp = rospy.Time(123, 456788000)
        child_stamp = rospy.Time(123, 456789000)
        parent_cloud = self._make_cloud("parent_frame", parent_stamp)
        child_cloud = self._make_cloud("child_frame", child_stamp)

        deadline = time.time() + 20.0
        while not rospy.is_shutdown() and time.time() < deadline:
            self.parent_pub.publish(parent_cloud)
            self.child_pub.publish(child_cloud)
            if self.output_event.wait(0.1):
                break

        self.assertTrue(self.output_event.is_set(), "Timed out waiting for calibrated cloud")
        self.assertEqual("parent_frame", self.output.header.frame_id)
        self.assertEqual(child_stamp.secs, self.output.header.stamp.secs)
        self.assertEqual(child_stamp.nsecs, self.output.header.stamp.nsecs)

        output_points = list(
            point_cloud2.read_points(
                self.output, field_names=("x", "y", "z", "intensity"), skip_nans=False
            )
        )
        expected_points = list(
            point_cloud2.read_points(
                child_cloud, field_names=("x", "y", "z", "intensity"), skip_nans=False
            )
        )

        self.assertEqual(len(expected_points), len(output_points))
        for expected, actual in zip(expected_points, output_points):
            for expected_value, actual_value in zip(expected[:3], actual[:3]):
                self.assertFalse(math.isnan(actual_value))
                self.assertFalse(math.isinf(actual_value))
                self.assertAlmostEqual(
                    expected_value,
                    actual_value,
                    delta=0.1,
                    msg="Expected identity transform, got {} vs {}".format(expected, actual),
                )
            self.assertAlmostEqual(expected[3], actual[3], delta=1e-6)

    @staticmethod
    def _make_cloud(frame_id, stamp):
        header = Header(stamp=stamp, frame_id=frame_id)
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("intensity", 16, PointField.FLOAT32, 1),
        ]
        points = []
        offsets = (0.15, 0.30, 0.45)
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


if __name__ == "__main__":
    rostest.rosrun(
        "multi_lidar_calibrator",
        "identity_calibration",
        IdentityCalibrationTest,
    )
