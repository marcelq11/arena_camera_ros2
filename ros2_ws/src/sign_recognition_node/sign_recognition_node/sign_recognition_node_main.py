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
        self.save_frames_and_signs = True
        self.mode_selector = 1 # 0 for full system, 1 for only sign detection
        self.vehicle_move = True #TODO: change to false after adding the vehicle movement detection
        self.previous_mode = self.mode_selector

        self.sign_text_recognition_system = SignTextRecognitionSystem(
            models_path=models_path,
            model_type='yolov8n',
            save_results=False,
            show_signs=False,
            show_images=False,
            save_signs=self.save_frames_and_signs,
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

    def mode_selector_handler(self):
        if not self.vehicle_move:
            self.previous_mode = self.mode_selector
            self.mode_selector = 2
        else:
            self.mode_selector = self.previous_mode

    def recognition_system_handler(self, cv_image, time_stamp):
        image = cv_image
        if self.run_system:
            
            self.mode_selector_handler()

            if self.mode_selector == 0:
                if self.enable_preview:
                    signs, text, image = self.sign_text_recognition_system.process_frame(cv_image, time_stamp)
                else :
                    signs, text = self.sign_text_recognition_system.process_frame(cv_image, time_stamp)
            elif self.mode_selector == 1:
                if self.enable_preview:
                    signs, frames, image = self.sign_text_recognition_system.frame_selector(cv_image, time_stamp)
                else:
                    signs, frames = self.sign_text_recognition_system.frame_selector(cv_image, time_stamp)
            elif self.mode_selector == 2:
                self.sign_text_recognition_system.process_sign_images(continue_processing=False)
            elif self.mode_selector == 3:
                self.sign_text_recognition_system.process_sign_images(continue_processing=True)

            #TODO: FIND EXAMPLE WHERE 2 signs are returned at the same time

            #Probably it will be deleted
            # if len(signs) > 0:
            #     sign = self.bridge.cv2_to_imgmsg(signs[0], "bgr8")
            #     self.publisher_sign.publish(sign)
            # if len(text) > 0:
            #     msg = String()
            #     msg.data = str(text)
            #     self.publisher_results.publish(msg)
        return image

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            time_stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        self.recognition_system_handler(cv_image, time_stamp)

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