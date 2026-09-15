"""Launch the calibrator with the i30 right-lidar example defaults."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_PARAMETER_TYPES = {
    "points_parent_src": str,
    "points_child_src": str,
    "voxel_size": float,
    "ndt_epsilon": float,
    "ndt_step_size": float,
    "ndt_resolution": float,
    "ndt_iterations": int,
    "x": float,
    "y": float,
    "z": float,
    "roll": float,
    "pitch": float,
    "yaw": float,
}


def _parameter_values():
    return {
        name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
        for name, value_type in _PARAMETER_TYPES.items()
    }


def generate_launch_description():
    argument_defaults = {
        "points_parent_src": "/center/velodyne_points",
        "points_child_src": "/right/velodyne_points",
        "voxel_size": "1.0",
        "ndt_epsilon": "0.01",
        "ndt_step_size": "0.1",
        "ndt_resolution": "1.0",
        "ndt_iterations": "400",
        "x": "0",
        "y": "-0.77",
        "z": "-0.3",
        "roll": "-3",
        "pitch": "2.6",
        "yaw": "1.59",
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(name, default_value=value)
            for name, value in argument_defaults.items()
        ]
        + [
            Node(
                package="multi_lidar_calibrator",
                executable="multi_lidar_calibrator",
                name="lidar_calibrator",
                output="screen",
                parameters=[_parameter_values()],
            )
        ]
    )
