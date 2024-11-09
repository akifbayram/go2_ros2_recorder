import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import cv2
from cv_bridge import CvBridge
import signal
import sys
from datetime import datetime, timedelta
import csv
import os

class VideoAndOdometryAndScanSaver(Node):
    def __init__(self):
        super().__init__('go2_ros2_recorder')

        # Declare and get parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('output_directory', './output'),
                ('image_topic', '/go2_camera/color/image'),
                ('odom_topic', '/odom'),
                ('scan_topic', '/scan'),
                ('video_filename', 'video.mp4'),
                ('odom_filename', 'odometry_data.csv'),
                ('scan_filename', 'scan_data.csv'),
                ('video_fps', 20.0),
                ('video_codec', 'mp4v'),
                ('display_video', True),
                ('check_interval', 5.0),
                ('no_data_timeout', 5.0)
            ]
        )

        self.output_directory = self.get_parameter('output_directory').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        video_filename = self.get_parameter('video_filename').get_parameter_value().string_value
        odom_filename = self.get_parameter('odom_filename').get_parameter_value().string_value
        scan_filename = self.get_parameter('scan_filename').get_parameter_value().string_value
        self.video_fps = self.get_parameter('video_fps').get_parameter_value().double_value
        self.video_codec = self.get_parameter('video_codec').get_parameter_value().string_value
        self.display_video = self.get_parameter('display_video').get_parameter_value().bool_value
        self.check_interval = self.get_parameter('check_interval').get_parameter_value().double_value
        self.no_data_timeout = self.get_parameter('no_data_timeout').get_parameter_value().double_value

        # Create output folder with timestamp
        folder_name = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_path = os.path.join(self.output_directory, folder_name)
        os.makedirs(self.output_path, exist_ok=True)
        self.get_logger().info(f'Output directory created at: {self.output_path}')

        # Initialize subscriptions
        self.bridge = CvBridge()
        self.out = None
        self.stop_recording = False
        self.last_frame_time = None

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

        # Recording flags
        self.is_recording_video = False
        self.is_recording_odom = False
        self.is_recording_scan = False

        # Signal handling
        self.shutdown_initiated = False
        signal.signal(signal.SIGINT, self.signal_handler)

        # Timer to check data reception
        self.timer = self.create_timer(self.check_interval, self.check_data_received)

    def listener_callback_image(self, data):
        if not self.stop_recording:
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
        try:
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
        try:
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
        if self.last_frame_time:
            time_since_last_frame = datetime.now() - self.last_frame_time
            if time_since_last_frame > timedelta(seconds=self.no_data_timeout):
                self.get_logger().warn(f'🔴 No video data received for over {self.no_data_timeout} seconds. Stopping recording...')
                self.initiate_shutdown()
        else:
            self.get_logger().warn('🟡 No video data received from image topic yet.')

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
    video_odometry_scan_saver = VideoAndOdometryAndScanSaver()
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
