import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, BoundingBox2DArray
import cv2
from cv_bridge import CvBridge


class MockImagePublisher(Node):
    def __init__(self, image_path):
        super().__init__('mock_image_publisher')

        if not os.path.exists(image_path):
            self.get_logger().error(f'image path {image_path} does not exist.')
            raise FileNotFoundError(f'image path {image_path} does not exist.')

        self.image_path = image_path

        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)

        self.bridge = CvBridge()

        self.cv_image = cv2.imread(self.image_path)

        self.timer = self.create_timer(1, self.publish_image)  # first parameter is T_seconds

    def publish_image(self):
        ros_image = self.bridge.cv2_to_imgmsg(self.cv_image, encoding='bgr8')

        self.publisher.publish(ros_image)

        self.get_logger().info('Published image')


def main(args=None):
    rclpy.init(args=args)

    image_path = '/home/user/src/image.jpg'

    try:
        node = MockImagePublisher(image_path)
    except FileNotFoundError:
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
