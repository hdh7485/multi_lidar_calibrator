# Multi LiDAR Calibrator

This package estimates the extrinsic calibration between two LiDAR point clouds with the Normal Distributions Transform (NDT) algorithm.

The `multi_lidar_calibrator` node synchronizes parent and child `sensor_msgs/msg/PointCloud2` messages, downsamples the child cloud for registration, and publishes the unfiltered child cloud transformed into the parent frame on `/points_calibrated`.

## ROS 2 Jazzy

The ROS 2 port targets **ROS 2 Jazzy on Ubuntu 24.04** and uses `ament_cmake`, C++17, PCL, and `message_filters` approximate-time synchronization.

### Build

Create a ROS 2 workspace, install dependencies, build, and source it:

```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/hdh7485/multi_lidar_calibrator.git
cd ..
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select multi_lidar_calibrator
source install/setup.bash
```

### Run

The node can be started directly with ROS 2 parameter overrides:

```sh
ros2 run multi_lidar_calibrator multi_lidar_calibrator --ros-args \
  -p points_parent_src:=/lidar_parent/points_raw \
  -p points_child_src:=/lidar_child/points_raw \
  -p x:=0.0 -p y:=0.0 -p z:=0.0 \
  -p roll:=0.0 -p pitch:=0.0 -p yaw:=0.0
```

The converted example launch files preserve the defaults from the ROS 1 XML launch files:

```sh
ros2 launch multi_lidar_calibrator multi_lidar_calibrator.launch.py
ros2 launch multi_lidar_calibrator multi_lidar_calibrator_flying_car.launch.py
ros2 launch multi_lidar_calibrator multi_lidar_calibrator_i30.launch.py
```

Every launch file accepts the same arguments as the original XML files, including `points_parent_src`, `points_child_src`, `voxel_size`, `ndt_epsilon`, `ndt_step_size`, `ndt_resolution`, `ndt_iterations`, `x`, `y`, `z`, `roll`, `pitch`, and `yaw`. The launch files start the executable with the historical `lidar_calibrator` launch name; direct execution uses the node name `multi_lidar_calibrator`.

Provide a useful initial guess for the sensor pair before starting a recording or rosbag. The node reports the estimated transform and prints a ROS 2 static transform command. The command arguments are translation followed by yaw, pitch, roll, parent frame, and child frame:

```sh
ros2 run tf2_ros static_transform_publisher \
  <x> <y> <z> <yaw> <pitch> <roll> <parent_frame> <child_frame>
```

### Parameters and topics

| Parameter | Type | Default | Description |
|---|---|---:|---|
| `points_parent_src` | string | `points_raw` | Parent point-cloud topic. |
| `points_child_src` | string | `points_raw` | Child point-cloud topic. |
| `voxel_size` | double | `0.1` | Voxel side length used to downsample only the child cloud before NDT. |
| `ndt_epsilon` | double | `0.01` | NDT transformation epsilon. |
| `ndt_step_size` | double | `0.1` | NDT line-search step size. |
| `ndt_resolution` | double | `1.0` | NDT target resolution. |
| `ndt_iterations` | integer | `400` | Maximum NDT iterations. |
| `x`, `y`, `z` | double | `0.0` | Initial translation guess in metres. |
| `roll`, `pitch`, `yaw` | double | `0.0` | Initial rotation guess in radians. |

The node subscribes to the two configured `PointCloud2` topics using approximate time synchronization. It converts messages to PCL `PointXYZI`, registers the downsampled child cloud against the parent cloud, transforms the original child cloud, and publishes `/points_calibrated`. The output frame is the parent frame and the output timestamp is the child cloud timestamp. Point count and intensity values are retained from the unfiltered child cloud.

### Regression test

The launch-testing regression test publishes identical synthetic parent and child clouds and verifies:

- output frame is the parent frame;
- output timestamp is the child timestamp;
- point count and intensity values are preserved; and
- output XYZ values remain near identity.

Run it with:

```sh
source /opt/ros/jazzy/setup.bash
colcon test --packages-select multi_lidar_calibrator --event-handlers console_direct+
colcon test-result --verbose
```

## Apple container verification

The following commands were used with Apple `container` on the host. The bind mount is the repository root, and the resource limit avoids the memory pressure seen with an unrestricted ROS 1 build.

ROS 1 baseline (`refactor/resource-ownership`; run this command while that branch is checked out):

```sh
container run --rm --memory 6G --cpus 4 \
  --volume "$PWD:/ws/src/multi_lidar_calibrator" \
  --workdir /ws ros:noetic-ros-base-focal bash -lc '
    set -euxo pipefail
    apt-get update
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      build-essential cmake libpcl-dev ros-noetic-pcl-ros \
      ros-noetic-pcl-conversions ros-noetic-message-filters \
      ros-noetic-rostest python3-rosunit
    catkin_make -j2
    source devel/setup.bash
    catkin_make run_tests_multi_lidar_calibrator -j2
    catkin_test_results
  '
```

ROS 2 Jazzy (`ros2`):

```sh
container run --rm --memory 6G --cpus 4 \
  --volume "$PWD:/ws/src/multi_lidar_calibrator" \
  --workdir /ws ros:jazzy-ros-base bash -lc '
    set -eo pipefail
    apt-get update
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      build-essential libpcl-dev ros-jazzy-pcl-conversions \
      ros-jazzy-message-filters ros-jazzy-launch-testing-ament-cmake \
      ros-jazzy-sensor-msgs-py
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install --packages-select multi_lidar_calibrator
    source install/setup.bash
    colcon test --packages-select multi_lidar_calibrator \
      --event-handlers console_direct+
    colcon test-result --verbose
  '
```

The ROS 1 baseline and ROS 2 port both pass their build and integration-test gates on Linux ARM64 with a 6 GB memory limit.

## License

Apache-2.0
