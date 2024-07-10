import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy
from cv_bridge import CvBridge
import cv2

qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            qos_profile)
        self.subscription

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            #self.get_logger().info('Received an image!')
            cv_image = cv2.resize(cv_image, (640, 640))
            cv2.imshow("Image", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error('Failed to convert image: ' + str(e))


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
