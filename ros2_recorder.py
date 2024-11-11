import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import cv2
from cv_bridge import CvBridge
import signal
from datetime import datetime, timedelta
import csv
import os
import argparse
import sys

class VideoAndOdometryAndScanSaver(Node):
    def __init__(self, args):
        super().__init__('go2_ros2_recorder')

        # Parse command-line arguments
        parser = argparse.ArgumentParser(description='Record video, odometry, and scan data.')
        parser.add_argument('--output_directory', type=str, default='./output', help='Directory to save output files.')
        parser.add_argument('--robot_type', type=str, choices=['go2', 'turtlebot4'], default='go2', help='Type of the robot.')
        parser.add_argument('--namespace', type=str, default='', help='Namespace of the robot (optional).')
        parser.add_argument('--image_topic', type=str, default='', help='Image topic to subscribe to.')
        parser.add_argument('--odom_topic', type=str, default='', help='Odometry topic to subscribe to.')
        parser.add_argument('--scan_topic', type=str, default='', help='Scan topic to subscribe to.')
        parser.add_argument('--video_filename', type=str, default='video.mp4', help='Filename for the recorded video.')
        parser.add_argument('--odom_filename', type=str, default='odometry_data.csv', help='Filename for odometry data.')
        parser.add_argument('--scan_filename', type=str, default='scan_data.csv', help='Filename for scan data.')
        parser.add_argument('--video_fps', type=float, default=20.0, help='Frames per second for the video.')
        parser.add_argument('--video_codec', type=str, default='mp4v', help='Codec for the video recording.')
        parser.add_argument('--display_video', action='store_true', help='Display video while recording.')
        parser.add_argument('--check_interval', type=float, default=5.0, help='Interval to check data reception (in seconds).')
        parser.add_argument('--no_data_timeout', type=float, default=5.0, help='Timeout for no data reception (in seconds).')

        parsed_args = parser.parse_args(args)

        # Initialize parameters and variables
        self.output_directory = parsed_args.output_directory
        self.robot_type = parsed_args.robot_type
        self.namespace = parsed_args.namespace.strip('/')
        image_topic_param = parsed_args.image_topic
        odom_topic_param = parsed_args.odom_topic
        scan_topic_param = parsed_args.scan_topic
        video_filename = parsed_args.video_filename
        odom_filename = parsed_args.odom_filename
        scan_filename = parsed_args.scan_filename
        self.video_fps = parsed_args.video_fps
        self.video_codec = parsed_args.video_codec
        self.display_video = parsed_args.display_video
        self.check_interval = parsed_args.check_interval
        self.no_data_timeout = parsed_args.no_data_timeout

        # Define default topics based on robot type
        robot_topic_defaults = {
            'go2': {
                'image_topic': '/go2_camera/color/image',
                'odom_topic': '/odom',
                'scan_topic': '/scan',
            },
            'turtlebot4': {
                'image_topic': '/4_0/oakd/rgb/preview/image_raw',
                'odom_topic': '/odom',
                'scan_topic': '/scan',
            }
        }

        # Check if the provided robot_type is supported
        if self.robot_type not in robot_topic_defaults:
            self.get_logger().warn(f"Unsupported robot_type '{self.robot_type}'. Falling back to 'go2' defaults.")
            self.robot_type = 'go2'

        # Get default topics based on robot type
        default_topics = robot_topic_defaults[self.robot_type]

        # Use provided topics or default
        image_topic = image_topic_param if image_topic_param else default_topics['image_topic']
        odom_topic = odom_topic_param if odom_topic_param else default_topics['odom_topic']
        scan_topic = scan_topic_param if scan_topic_param else default_topics['scan_topic']

        # Prefix namespace if provided
        if self.namespace:
            image_topic = f'/{self.namespace}/{image_topic.lstrip("/")}'
            odom_topic = f'/{self.namespace}/{odom_topic.lstrip("/")}'
            scan_topic = f'/{self.namespace}/{scan_topic.lstrip("/")}'
        else:
            image_topic = image_topic if image_topic.startswith('/') else f'/{image_topic}'
            odom_topic = odom_topic if odom_topic.startswith('/') else f'/{odom_topic}'
            scan_topic = scan_topic if scan_topic.startswith('/') else f'/{scan_topic}'

        # Create output folder with timestamp
        folder_name = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_path = os.path.join(self.output_directory, folder_name)
        os.makedirs(self.output_path, exist_ok=True)
        self.get_logger().info(f'Output directory created at: {self.output_path}')

        # Initialize subscription flags
        self.bridge = CvBridge()
        self.out = None
        self.stop_recording = False
        self.last_frame_time = None
        self.last_odom_time = None
        self.last_scan_time = None

        # Initialize recording flags
        self.is_recording_video = False
        self.is_recording_odom = False
        self.is_recording_scan = False

        # Signal handling
        self.shutdown_initiated = False
        signal.signal(signal.SIGINT, self.signal_handler)

        # Video setup
        self.file_name = os.path.join(self.output_path, video_filename)
        self.get_logger().info(f'🟡 Video recording started. Saving to file: {self.file_name}.')
        self.get_logger().info('Press "Ctrl+C" to stop recording.')
        self.get_logger().info(f'🟡 Waiting for video data from {image_topic}...')

        self.subscription_image = self.create_subscription(
            Image,
            image_topic,
            self.listener_callback_image,
            QoSProfile(depth=10)
        )

        # Odometry setup
        self.subscription_odometry = self.create_subscription(
            Odometry,
            odom_topic,
            self.listener_callback_odometry,
            QoSProfile(depth=10)
        )

        self.odom_file_name = os.path.join(self.output_path, odom_filename)
        self.get_logger().info(f'🟡 Subscribed to odometry data at {odom_topic}. Saving to file: {self.odom_file_name}.')
        self.csv_file_odom = open(self.odom_file_name, mode='w', newline='')
        self.csv_writer_odom = csv.writer(self.csv_file_odom)
        self.csv_writer_odom.writerow(['timestamp', 'x', 'y', 'z'])

        # Scan setup with Best Effort reliability
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.subscription_scan = self.create_subscription(
            LaserScan,
            scan_topic,
            self.listener_callback_scan,
            qos_profile
        )

        self.scan_file_name = os.path.join(self.output_path, scan_filename)
        self.get_logger().info(f'🟡 Subscribed to scan data at {scan_topic}. Saving to file: {self.scan_file_name}.')
        self.csv_file_scan = open(self.scan_file_name, mode='w', newline='')
        self.csv_writer_scan = csv.writer(self.csv_file_scan)
        self.csv_writer_scan.writerow([
            'timestamp_sec', 'timestamp_nanosec', 'angle_min', 'angle_max',
            'angle_increment', 'range_min', 'range_max', 'ranges'
        ])

        # Timer to check data reception
        self.timer = self.create_timer(self.check_interval, self.check_data_received)

    def listener_callback_image(self, data):
        if self.stop_recording:
            return

        try:
            self.last_frame_time = datetime.now()

            if not self.is_recording_video:
                self.get_logger().info('🟢 Video data received! Recording video in progress.')
                self.is_recording_video = True

            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            if self.out is None:
                height, width = cv_image.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*self.video_codec)
                self.out = cv2.VideoWriter(self.file_name, fourcc, self.video_fps, (width, height))
                if not self.out.isOpened():
                    self.get_logger().error(f'Failed to open video writer with codec {self.video_codec}')
                    self.stop_recording = True
                    return
                self.get_logger().info(f'🟢 Video writer initialized: {width}x{height}, {self.video_fps} FPS.')

            self.out.write(cv_image)

            if self.display_video:
                cv2.imshow("Frame", cv_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.get_logger().info('🟢 "q" pressed. Exiting video display.')
                    self.display_video = False

        except Exception as e:
            self.get_logger().error(f'Error processing frame: {e}')

    def listener_callback_odometry(self, msg):
        if self.stop_recording:
            return

        try:
            self.last_odom_time = datetime.now()

            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z

            self.csv_writer_odom.writerow([timestamp, x, y, z])

            if not self.is_recording_odom:
                self.get_logger().info('🟢 Odometry data received! Recording odometry data in progress.')
                self.is_recording_odom = True

        except Exception as e:
            self.get_logger().error(f'Error processing odometry data: {e}')

    def listener_callback_scan(self, msg):
        if self.stop_recording:
            return

        try:
            self.last_scan_time = datetime.now()

            timestamp_sec = msg.header.stamp.sec
            timestamp_nanosec = msg.header.stamp.nanosec
            angle_min = msg.angle_min
            angle_max = msg.angle_max
            angle_increment = msg.angle_increment
            range_min = msg.range_min
            range_max = msg.range_max
            ranges = ';'.join(map(str, msg.ranges))  # Convert list to semicolon-separated string

            self.csv_writer_scan.writerow([
                timestamp_sec, timestamp_nanosec, angle_min, angle_max,
                angle_increment, range_min, range_max, ranges
            ])

            if not self.is_recording_scan:
                self.get_logger().info('🟢 Lidar scan data received! Recording scan data in progress.')
                self.is_recording_scan = True

        except Exception as e:
            self.get_logger().error(f'Error processing scan data: {e}')

    def check_data_received(self):
        current_time = datetime.now()

        # Check for video data
        if self.last_frame_time:
            time_since_last_frame = current_time - self.last_frame_time
            if time_since_last_frame > timedelta(seconds=self.no_data_timeout):
                self.get_logger().warn(f'🔴 No video data received for over {self.no_data_timeout} seconds.')
                self.initiate_shutdown()
        else:
            self.get_logger().warn('🟡 No video data received from image topic yet.')

        # Check for odometry data
        if self.last_odom_time:
            time_since_last_odom = current_time - self.last_odom_time
            if time_since_last_odom > timedelta(seconds=self.no_data_timeout):
                self.get_logger().warn(f'🔴 No odometry data received for over {self.no_data_timeout} seconds.')
                self.initiate_shutdown()
        else:
            self.get_logger().warn('🟡 No odometry data received from odometry topic yet.')

        # Check for scan data
        if self.last_scan_time:
            time_since_last_scan = current_time - self.last_scan_time
            if time_since_last_scan > timedelta(seconds=self.no_data_timeout):
                self.get_logger().warn(f'🔴 No scan data received for over {self.no_data_timeout} seconds.')
                self.initiate_shutdown()
        else:
            self.get_logger().warn('🟡 No scan data received from scan topic yet.')

    def initiate_shutdown(self):
        if not self.shutdown_initiated:
            self.shutdown_initiated = True
            self.stop_recording = True
            self.get_logger().info('🔴 Stopping recording...')
            self.cleanup_resources()

    def cleanup_resources(self):
        # Release video writer
        if self.out:
            self.out.release()
            self.get_logger().info(f'🔴 Video recording stopped. File saved: {self.file_name}')

        # Close CSV files
        if self.csv_file_odom:
            self.csv_file_odom.close()
            self.get_logger().info(f'🔴 Odometry data saved to CSV file: {self.odom_file_name}')

        if self.csv_file_scan:
            self.csv_file_scan.close()
            self.get_logger().info(f'🔴 Scan data saved to CSV file: {self.scan_file_name}')

        # Destroy OpenCV windows
        if self.display_video:
            cv2.destroyAllWindows()

        # Destroy node (without calling rclpy.shutdown())
        self.destroy_node()

    def signal_handler(self, sig, frame):
        self.get_logger().info('🔴 Ctrl+C detected, stopping recording...')
        self.initiate_shutdown()

def main(args=None):
    rclpy.init(args=args)
    video_odometry_scan_saver = VideoAndOdometryAndScanSaver(sys.argv[1:])
    try:
        rclpy.spin(video_odometry_scan_saver)
    except KeyboardInterrupt:
        video_odometry_scan_saver.get_logger().info('🔴 KeyboardInterrupt detected, shutting down...')
    finally:
        if not video_odometry_scan_saver.shutdown_initiated:
            video_odometry_scan_saver.initiate_shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
