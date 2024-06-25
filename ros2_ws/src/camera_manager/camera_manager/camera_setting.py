# camera_publisher.py
import rclpy
from rclpy.node import Node
from .setting_gui import CameraGUI
from camera_msg.msg import CameraSettings
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy
import cv2

qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
path_to_camera_params ='/home/opszalek/arena_camera_ros2/ros2_ws/src/arena_camera_node/config'#path to camera parameters in arena_camera_node
class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(CameraSettings, 'params', 10)
        self.gui = CameraGUI(self.update_settings, )  # Przekazanie callback
        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            qos_profile)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.get_logger().info('Received an image!')
            # Display the image with OpenCV
            cv2.imshow("Image window", cv_image)
            cv2.moveWindow("Image window", 500, 500)
            results = self.model(cv_image)
            annotated_frame = results[0].plot()

            # Display the annotated frame
            annotated_frame = cv2.resize(annotated_frame, (640, 640))
            cv2.imshow("YOLOv8 Tracking", annotated_frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error('Failed to convert image: ' + str(e))

    def update_settings(self):
        gain, exposure, gamma, pixel_format, width, height = self.gui.get_settings()
        msg = CameraSettings()
        msg.exposure_time = exposure
        msg.gamma = gamma
        msg.gain = gain
        print("______________________________________________")
        msg.pixelformat = pixel_format
        msg.width = width
        msg.height = height
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg}')

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
