import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np
from ultralytics import YOLO

class YoloFollower(Node):
    def __init__(self):
        super().__init__('yolo_follower')

        self.subscriber = self.create_subscription(
            Image, '/fishbot_camera_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)


        self.model = YOLO('best.pt')

        self.bridge = CvBridge()

        self.target_label = 0
        self.speed = 0.5

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, _ = image.shape

        results = self.model(image, verbose=False, conf=0.6)[0]

        targets = [det.cpu().numpy() for det in results.boxes.data if int(det[-1]) == self.target_label]

        if targets:
            closest_target = min(
                targets,
                key=lambda box: np.linalg.norm(
                    np.array([(box[0]+box[2])/2 - width / 2, (box[1]+box[3])/2 - height / 2])
                )
            )

            center_x = (closest_target[0] + closest_target[2]) / 2

            error_x = center_x - width / 2

            twist = Twist()
            twist.linear.x = self.speed
            twist.angular.z = -float(error_x) / (width / 2)

            self.get_logger().info(f'Target found, moving with speed {self.speed:.2f}, angular {twist.angular.z:.2f}')
        else:
            twist = Twist()
            self.get_logger().info('No target found, stopping.')

        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    yolo_follower = YoloFollower()
    rclpy.spin(yolo_follower)
    yolo_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
