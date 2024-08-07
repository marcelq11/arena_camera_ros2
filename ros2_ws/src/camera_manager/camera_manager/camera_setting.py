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
                             error_func=self.raise_error, parameters_path=path_to_camera_params)  # Przekazanie callback

    def update_settings(self):
        gain, exposure, gamma, pixel_format, width, height, recording = self.gui.return_params()
        msg = CameraSettings()
        msg.exposure_time = exposure
        msg.gamma = gamma
        msg.gain = gain
        msg.pixelformat = pixel_format
        msg.width = width
        msg.height = height
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
