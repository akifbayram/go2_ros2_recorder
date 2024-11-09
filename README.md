**go2_ros2_recorder** is a script designed to work with the [go2_ros2_sdk](https://github.com/akifbayram/go2_ros2_sdk) for the Unitree Go2 robot. It records video streams from the robot's camera, along with odometry and LIDAR scan data, saving them into organized and timestamped files for easy analysis and processing. Optimized for single-robot setups but can be configured via parameters for a namespaced robot.

![image](image.png)

## Tested Prerequisites

- **Operating System**: Ubuntu 22.04
- **ROS2 Distribution**: Humble
- **go2_ros2_sdk**: Ensure the [go2_ros2_sdk](https://github.com/yourusername/go2_ros2_sdk) is installed and properly configured for your Unitree Go2 robot.
- **Python Dependencies**:
  - OpenCV (`opencv-python`)
  - `cv_bridge`
  - `rclpy`

## Installation

**Clone the Repository**

   ```bash
   git clone https://github.com/akifbayram/go2_ros2_recorder.git
   cd go2_ros2_recorder
   ```

## Usage

1. **Ensure `go2_ros2_sdk` is Running**

   Make sure the `go2_ros2_sdk` is active and publishing the necessary topics.

2. **Run the Script**

   ```bash
   python go2_ros2_recorder.py
   ```

3. **Configure Parameters (Optional)**

   You can customize the script's behavior using command-line arguments or a configuration file.

   **Using Command-Line Arguments:**

   ```bash
   python go2_ros2_recorder.py \
     --output_directory "/home/user/ros_output" \
     --video_fps 25.0 \
     --display_video false
   ```

   **Using a Configuration File:**

   Create a `config.yaml`:

   ```yaml
   output_directory: "/home/user/ros_output"
   image_topic: "/go2_camera/color/image"
   odom_topic: "/odom"
   scan_topic: "/scan"
   video_fps: 30.0
   video_codec: "mp4v"
   display_video: true
   no_data_timeout: 5.0
   ```

   Run the script with the config file:

   ```bash
   python go2_ros2_recorder.py --config config.yaml
   ```

4. **Stop Recording**

   Press `Ctrl+C` in the terminal to stop recording.

## Acknowledgments

This project utilizes the excellent work from the following repositories:

- [tfoldi/go2-webrtc](https://github.com/tfoldi/go2-webrtc)
- [legion1581/go2_webrtc_connect](https://github.com/legion1581/go2_webrtc_connect)