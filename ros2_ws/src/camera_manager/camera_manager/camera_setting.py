# camera_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .setting_gui import CameraGUI

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(String, 'camera_params', 10)
        self.gui = CameraGUI(self.update_settings)  # Przekazanie callback

    def update_settings(self):
        gain, exposure, gamma, pixel_format = self.gui.get_settings()
        message = f"GAIN: {gain}, EXPOSURE: {exposure}, GAMMA: {gamma}, PIXEL FORMAT: {pixel_format}"
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

    def run(self):
        rclpy.spin_once(self, timeout_sec=0.1)
        self.gui.run()  # Uruchomienie GUI

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    camera_publisher.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
