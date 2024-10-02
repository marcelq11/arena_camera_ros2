import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import QoSProfile, ReliabilityPolicy
import cv2
import os
import time
import threading
import queue
from image_processing.image_main import SignTextRecognitionSystem
from camera_msg.msg import CameraSettings

qos_profile = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)

class SignTextRecognitionNode(Node):
    def __init__(self):
        super().__init__('sign_text_recognition_node')

        self.system_restarted = True
        self.bridge = CvBridge()
        self.image_queue = queue.Queue()

        models_path = os.environ.get('MODELS_PATH')

        self.enable_preview = True
        self.run_system = False

        self.sign_text_recognition_system = SignTextRecognitionSystem(
            models_path=models_path,
            model_type='yolov8n',
            save_results=False,
            show_signs=False,
            show_images=False,
            save_frames=False,
            enable_preview=self.enable_preview,
            ocr='paddle'
        )

        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            qos_profile)

        self.param_subscriber = self.create_subscription(
            CameraSettings,
            '/params',
            self.param_callback,
            qos_profile)


        self.publisher_results = self.create_publisher(String, 'sign_text_output', 10)
        self.publisher_sign = self.create_publisher(Image, '/sign', 10)
        self.previous_time = time.time()
        self.sum_time = 0
        self.frame_count = 0


        self.display_thread = threading.Thread(target=self.display_images)
        self.display_thread.daemon = True
        self.display_thread.start()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting image: {e}')
            return
        
        if self.run_system:
            if self.enable_preview:
                signs, text, image = self.sign_text_recognition_system.process_frame(cv_image)
            else :
                signs, text = self.sign_text_recognition_system.process_image(cv_image)

            #TODO: FIND EXAMPLE WHERE 2 signs are returned at the same time
            if len(signs) > 0:
                sign = self.bridge.cv2_to_imgmsg(signs[0], "bgr8")
                self.publisher_sign.publish(sign)
            if len(text) > 0:
                msg = String()
                msg.data = str(text)
                self.publisher_results.publish(msg)


            current_time = time.time()
            frame_duration = current_time - self.previous_time

            #FPS calculation FOR TESTING
            # self.sum_time += frame_duration
            # self.frame_count += 1
            # if frame_duration > 0:
            #     fps = 1.0 / frame_duration
            #     avg_fps = self.frame_count / self.sum_time
            #     self.get_logger().info(f'FPS: {fps:.2f}, Average FPS: {avg_fps:.2f}')

            self.previous_time = current_time

            if self.enable_preview:
                image = cv2.resize(image, (640, 640))
                self.image_queue.put(image)

            self.system_restarted = False

    def display_images(self):
        while rclpy.ok():
            try:
                cv_image = self.image_queue.get(timeout=1)
                if cv_image is not None:
                    cv2.imshow("Image", cv_image)
                    cv2.waitKey(1)
            except queue.Empty:
                continue

    def param_callback(self, msg):
        self.run_system = msg.system_start
        if not self.run_system and not self.system_restarted:
            self.sign_text_recognition_system.reset_system()
            self.system_restarted = True

def main(args=None):
    rclpy.init(args=args)
    node = SignTextRecognitionNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()