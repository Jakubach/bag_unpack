# bag_unpack

`bag_unpack` is a ROS 2 (Robot Operating System) utility package that processes ROS 2 bag files (`.db3`) by filtering. The package allows you to filter out specific topics or transformations from `/tf` and `/tf_static` topics, and write the filtered data to a new bag file.

## Features

- Filter out specific transformations in `/tf` and `/tf_static` topics using user-defined exclusions.
- Select which topics to include in the output bag file.
- Currently supports reading and writing ROS 2 bag files with SQLite3 storage backend.

## Prerequisites

- ROS 2 (Humble or newer)
- Python 3.10 or newer
- Dependencies:
  - `rosbag2_py`
  - `tf2_msgs`
  - `ament_index_python`

## Installation

1. **Clone the repository**:
    ```bash
    git clone https://github.com/jakubach/bag_unpack.git
    cd bag_unpack
    ```

2. **Install dependencies**:
    If you don't already have the required dependencies, you can install them via `rosdep`:
    ```bash
    rosdep install --from-paths src -y --ignore-src --rosdistro humble
    ```

3. **Build the package**:
    ```bash
    colcon build
    ```

4. **Source the workspace**:
    ```bash
    source install/setup.bash
    ```
5. **Launch**:
    ```bash
    ros2 launch bag_unpack bag_unpack_launch.py
    ```

## Configuration

Before running the application, specify the parameters in a YAML configuration file. The configuration file should be located in the `config` directory of the package. Transformations are specified according to the scheme: `frame_id:2:child_frame_id`.

Here is an example configuration (`config/config.yaml`):

```yaml
bag_unpack:
  ros__parameters:
    input_bag_path: "/path/to/your/bag_input_dir"
    output_bag_path: "/path/to/your/bag_output_dir"
    included_topics:
        - "/camera/image"
        - "/imu/data"
        - "/odom"
        - "/tf"
        - "/tf_static"
    excluded_tf:
        - "map:2:odom"
        - "base_link:2:camera_link"
