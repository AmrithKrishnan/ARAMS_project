import os
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            'traffic_light_roi',  # input image topic
            self.image_callback,
            100
        )
        self.publisher = self.create_publisher(Image, 'traffic_light_cropped', 10)
        self.truck_publisher = self.create_publisher(String, '/truck_status', 10)

        # Load YOLO model weights and configuration
        package_path = os.path.dirname(os.path.realpath(__file__))
        weights_file = os.path.abspath(os.path.join(package_path, 'data', 'weights', 'yolov4-tiny.weights'))
        config_file = os.path.abspath(os.path.join(package_path, 'data', 'cfg', 'yolov4-tiny.cfg'))

        print("Weights file path:", weights_file)
        print("Config file path:", config_file)

        if not os.path.exists(weights_file):
            print("ERROR: weights file not found.")
        if not os.path.exists(config_file):
            print("ERROR: config file not found.")

        # Load class names used by YOLO
        classes_file = os.path.join(package_path, 'data', 'coco.names')
        with open(classes_file, 'rt') as f:
            self.classes = f.read().rstrip('\n').split('\n')

        # Load YOLO model
        self.net = cv2.dnn_DetectionModel(config_file, weights_file)
        self.net.setInputSize(320, 128)  # Set the input size (width, height) for the YOLO model. have to be multiples of 32
        self.net.setInputScale(1.0 / 255)  # Set the input scale for the YOLO model
        self.net.setInputSwapRB(True)  # Set to True to swap Blue and Red channels (OpenCV uses BGR by default)

    def image_callback(self, msg):
        # Convert the Image message to an OpenCV image with rgb8 encoding
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Run YOLO on the preprocessed image to detect traffic lights and trucks
        classes, scores, boxes = self.net.detect(cv_image, confThreshold=0.5, nmsThreshold=0.4)

        # Process YOLO detections
        traffic_light_crops = []
        for class_id, score, box in zip(classes, scores, boxes):
            if self.classes[class_id] == 'traffic light':
                x, y, w, h = box
                traffic_light_crop = cv_image[y:y+h, x:x+w]
                traffic_light_crops.append(traffic_light_crop)

            # Check for the class 'truck' and publish '1' if found
            if self.classes[class_id] == 'truck':
                truck_status_msg = String()
                truck_status_msg.data = "1"
                self.truck_publisher.publish(truck_status_msg)

        # Publish the cropped images
        for idx, traffic_light_crop in enumerate(traffic_light_crops):
            ros_frame = self.bridge.cv2_to_imgmsg(traffic_light_crop, encoding="passthrough")
            self.publisher.publish(ros_frame)

def main(args=None):
    rclpy.init(args=args)
    traffic_light_detector = TrafficLightDetector()
    rclpy.spin(traffic_light_detector)
    traffic_light_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
