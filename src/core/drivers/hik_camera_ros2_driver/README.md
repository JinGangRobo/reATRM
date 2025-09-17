[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build](https://github.com/SMBU-PolarBear-Robotics-Team/hik_camera_ros2_driver/actions/workflows/ci.yml/badge.svg)](https://github.com/SMBU-PolarBear-Robotics-Team/hik_camera_ros2_driver/actions/workflows/ci.yml)

# hik_camera_ros2_driver

## Overview

The `hik_camera_ros2_driver` package provides a ROS 2 driver for controlling and interfacing with Hikvision cameras. It supports functionalities such as camera initialization, parameter configuration, and image publishing. This package is intended for applications requiring reliable and configurable image data acquisition in a ROS 2 environment.

### Executables

The package includes the `hik_camera_node`, which manages the camera and publishes image data along with camera information to ROS 2 topics.

### Subscribed Topics

None.

### Published Topics

- `<camera_topic>` (sensor_msgs/msg/Image)
  - The image data captured by the Hikvision camera.

- `<camera_topic>/camera_info` (sensor_msgs/msg/CameraInfo)
  - Camera calibration information.

### Parameters

- `exposure_time` (double, default: `5000`)
  - The camera exposure time in microseconds.

- `gain` (double, default: `camera`)
  - The gain setting for the camera.

- `acquisition_frame_rate` (double, default: `165`)
  - The acquisition frame rate in hz for the camera.

- `pixel_format` (string, default: `RGB8Packed`)
  - The pixel format for the image data. Supported values: `Mono8`, `Mono10`, `Mono12`, `RGB8Packed`, `BGR8Packed`, `YUV422_YUYV_Packed`, `YUV422Packed`, `BayerRG8`, `BayerRG10`, `BayerRG10Packed`, `BayerRG12`, `BayerRG12Packed`.

- `adc_bit_depth` (string, default: `Bits_8`)
  - The ADC bit depth for the camera. Supported values: `Bits_8`, `Bits_12`.

- `use_sensor_data_qos` (bool, default: true)
  - Whether to use the `sensor_data` QoS profile for image topic publication.

- `camera_name` (string, default: `camera`)
  - The name of the camera for identification purposes.

- `frame_id` (string, default: `<camera_name>_optical_frame`)
  - The frame_id assigned to the published image data.

- `camera_topic` (string, default: `<camera_name>/image`)
  - The topic name for publishing image and info data.

- `camera_info_url` (string, default: `package://hik_camera_ros2_driver/config/camera_info.yaml`)
  - The URL for the camera calibration information file.

### Usage

#### Installation

To use this package, build it from source or include it in your ROS 2 workspace. Ensure that all dependencies are installed. You **don't** need to install the Hikvision camera SDK and include its libraries in your environment.

```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
```

```bash
git clone https://github.com/SMBU-PolarBear-Robotics-Team/hik_camera_ros2_driver.git
```

```bash
cd ~/ros_ws
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

#### Run

You can use the provided launch file for starting the camera node with default or custom parameters:

```bash
ros2 launch hik_camera_ros2_driver hik_camera_launch.py
```
