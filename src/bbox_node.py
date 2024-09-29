import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, BoundingBox2DArray
import cv2
from cv_bridge import CvBridge

class MultiBoundingBoxNode(Node):
    def __init__(self):
        super().__init__('multi_bounding_box_node')

        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        self.publisher = self.create_publisher(BoundingBox2DArray, '/vision/bounding_boxes', 10)

        self.bridge = CvBridge()


    def image_callback(self, msg):
        current_frame  = self.bridge.imgmsg_to_cv2(msg)

        gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        bounding_boxes = BoundingBox2DArray()

        if contours:
            for contour in contours:
                x,y,w,h = cv2.boundingRect(contour)

                bounding_box = BoundingBox2D()
                bounding_box.center.position.x = x + w/2.0
                bounding_box.center.position.y = y + h/2.0
                bounding_box.size_x = float(w)
                bounding_box.size_y = float(h)

                bounding_boxes.boxes.append(bounding_box)

            self.publisher.publish(bounding_boxes)

            self.get_logger().info(f'Published {len(bounding_boxes.boxes)} bounding boxes.')
        else:
            self.get_logger().info('No contours found.')


def main(args=None):
    rclpy.init(args=args)
    node = MultiBoundingBoxNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()