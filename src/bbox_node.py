import rclpy
from rclpy.node import Node
from ultralytics import YOLO
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, BoundingBox2DArray


class MultiBoundingBoxNode(Node):
    def __init__(self):
        super().__init__('multi_bounding_box_node')

        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        self.publisher = self.create_publisher(BoundingBox2DArray, '/vision/bounding_boxes', 10)

        self.bridge = CvBridge()

        # Set the path to the trained model (change to your model path)
        model_path = '/home/user/weights/best.engine'  # You can use 'yolov8n-seg.pt' or any trained YOLOv8 model
        
        # Load the YOLOv8 segmentation model
        self.model = YOLO(model_path)
        # self.model.to('cuda')
        self.class_names = ['Sky', 'Water', 'Bridge', 'Obstacle', 'Living Obstacle', 'Background', 'Self']
        # 



    def image_callback(self, msg):
        current_frame  = self.bridge.imgmsg_to_cv2(msg)

        results = self.model(current_frame)

        boxes = results[0].boxes.xyxy.cpu().numpy()  # Bounding boxes as (x1, y1, x2, y2)
        class_ids = results[0].boxes.cls.cpu().numpy()  # Class IDs
        confidences = results[0].boxes.conf.cpu().numpy()  # Confidence scores
        xywh_bboxes = []
        # Iterate over each bounding box and its corresponding class
        for i, box in enumerate(boxes):
            class_id = int(class_ids[i])
            confidence = confidences[i]
            class_name = self.class_names[class_id]
            if class_name != "Obstacle" or confidence < 0.5:
                continue        
            # Get the box coordinates
            x1, y1, x2, y2 = map(float, box)
            w = x2-x1
            h = y2-y1
            xywh_bboxes.append([x1,y1,w,h])


        if xywh_bboxes:
            bounding_boxes = BoundingBox2DArray()

            for x,y,w,h in xywh_bboxes:
                bounding_box = BoundingBox2D()
                bounding_box.center.position.x = x + w/2.0
                bounding_box.center.position.y = y + h/2.0
                bounding_box.size_x = float(w)
                bounding_box.size_y = float(h)

                bounding_boxes.boxes.append(bounding_box)

                            
                print(f"Detected {class_name} with confidence {confidence:.2f}")

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