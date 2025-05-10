import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageRecorder(Node):
    def __init__(self):
        super().__init__('image_recorder')

        self.subscription = self.create_subscription(
            Image,
            '/fishbot_camera_raw',
            self.listener_callback,
            10)

        self.bridge = CvBridge()
        self.save_interval = 0.5  # 秒
        self.img_dir = 'img'

        if not os.path.exists(self.img_dir):
            os.makedirs(self.img_dir)

        self.img_count = 1
        self.last_saved_time = self.get_clock().now()

    def listener_callback(self, msg):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.last_saved_time).nanoseconds / 1e9

        if elapsed_time >= self.save_interval:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            img_path = os.path.join(self.img_dir, f'{self.img_count}.png')
            cv2.imwrite(img_path, cv_image)

            self.get_logger().info(f'Saved image {img_path}')

            self.img_count += 1
            self.last_saved_time = current_time


def main(args=None):
    rclpy.init(args=args)
    image_recorder = ImageRecorder()
    rclpy.spin(image_recorder)
    image_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    