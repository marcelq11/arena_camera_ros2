# camera_publisher.py
import rclpy
import os
from rclpy.node import Node
from .setting_gui import CameraGUI
from camera_msg.msg import CameraSettings
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory

qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
path_to_camera_params = os.path.join(get_package_share_directory('arena_camera_node'), 'config', 'arena_camera_params.yaml')  #path to camera parameters in arena_camera_node
#TODO: fix path to camera parameters in arena_camera_node because it takes it from install directory not from src directory
class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(CameraSettings, 'params', 10)
        self.gui = CameraGUI(update_callback=self.update_settings,
                             error_func=self.raise_error, parameters_path=path_to_camera_params)

    def update_settings(self):
        exposure_limits, gain_limits, brightness, roi, resolution, system_start, recording = self.gui.return_parameters()
        msg = CameraSettings()
        msg.exposure_time_upper_limit = float(exposure_limits[1])
        msg.exposure_time_lower_limit = float(exposure_limits[0])
        msg.gain_lower_limit = float(gain_limits[0])
        msg.gain_upper_limit = float(gain_limits[1])
        msg.target_brightness = int(brightness)
        msg.roi_width = int(roi[0])
        msg.roi_height = int(roi[1])
        msg.roi_offset_x = int(roi[2])
        msg.roi_offset_y = int(roi[3])
        msg.width = int(resolution[0])
        msg.height = int(resolution[1])
        msg.system_start = system_start
        msg.recording = recording
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg}')

    def raise_error(self, e):
        self.get_logger().warn(e)

    def run(self):
        rclpy.spin_once(self, timeout_sec=0.1)
        self.gui.run()


def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    camera_publisher.run()
    camera_publisher.gui.root.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
