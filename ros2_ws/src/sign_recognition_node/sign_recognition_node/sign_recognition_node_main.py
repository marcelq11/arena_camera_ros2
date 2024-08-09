import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import QoSProfile, ReliabilityPolicy
import cv2
import os

from image_processing.image_main import SignTextRecognitionSystem

qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

class SignTextRecognitionNode(Node):
    def __init__(self):
        super().__init__('sign_text_recognition_node')

        self.bridge = CvBridge()

        models_path = os.environ.get('MODELS_PATH')

        self.sign_text_recognition_system = SignTextRecognitionSystem(
            models_path=models_path,
            model='yolov8',
            save_results=True,
            show_signs=True,
            show_images=False,
            save_frames=False,
            ocr='paddle'
        )

        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            qos_profile)
        self.subscription

        self.publisher = self.create_publisher(String, 'sign_text_output', 10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        self.sign_text_recognition_system.process_frame(cv_image)
        
        text_output = "Sample Text Output"
        msg = String()
        msg.data = text_output
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SignTextRecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
