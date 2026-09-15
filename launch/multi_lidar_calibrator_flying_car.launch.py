"""Launch the calibrator with the flying-car example defaults."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _parameter_values():
    return {
        "points_parent_src": ParameterValue(
            LaunchConfiguration("points_parent_src"), value_type=str
        ),
        "points_child_src": ParameterValue(
            LaunchConfiguration("points_child_src"), value_type=str
        ),
        "voxel_size": ParameterValue(
            LaunchConfiguration("voxel_size"), value_type=float
        ),
        "ndt_epsilon": ParameterValue(
            LaunchConfiguration("ndt_epsilon"), value_type=float
        ),
        "ndt_step_size": ParameterValue(
            LaunchConfiguration("ndt_step_size"), value_type=float
        ),
        "ndt_resolution": ParameterValue(
            LaunchConfiguration("ndt_resolution"), value_type=float
        ),
        "ndt_iterations": ParameterValue(
            LaunchConfiguration("ndt_iterations"), value_type=int
        ),
        "x": ParameterValue(LaunchConfiguration("x"), value_type=float),
        "y": ParameterValue(LaunchConfiguration("y"), value_type=float),
        "z": ParameterValue(LaunchConfiguration("z"), value_type=float),
        "roll": ParameterValue(LaunchConfiguration("roll"), value_type=float),
        "pitch": ParameterValue(LaunchConfiguration("pitch"), value_type=float),
        "yaw": ParameterValue(LaunchConfiguration("yaw"), value_type=float),
    }


def generate_launch_description():
    argument_defaults = {
        "points_parent_src": "/velodyne_points",
        "points_child_src": "/camera/depth_registered/points",
        "voxel_size": "1.0",
        "ndt_epsilon": "0.01",
        "ndt_step_size": "0.1",
        "ndt_resolution": "1.0",
        "ndt_iterations": "100",
        "x": "0.3",
        "y": "0",
        "z": "-0.15",
        "roll": "0",
        "pitch": "0.349",
        "yaw": "0",
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
