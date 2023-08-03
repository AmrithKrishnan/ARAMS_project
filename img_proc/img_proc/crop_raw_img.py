import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge
import numpy 

class MyNode(Node):
    def __init__(self):
        super().__init__('crop_raw_img')
        self.publisher_ = self.create_publisher(Image, '/traffic_light_roi', 10)
        self.subscription_ = self.create_subscription(
            Image, '/prius/front_camera/image_raw', self.callback, 10)
        self.subscription_
        
        self.cv_frame = None  # Instance variable

    def callback(self, msg):
        self.get_logger().info('Received an image message')
        
        # Convert ROS2 Image message to OpenCV image
        cv_bridge = CvBridge()
        cv_frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Process the received message here
        cropped_img_bgr = cv_frame[190:510, 600:728] # OLD PARA 200-500, 600-725
        cropped_img_rgb = cv.cvtColor(cropped_img_bgr, cv.COLOR_BGR2RGB)

        # Convert OpenCV image to ROS2 Image
        ros_frame = cv_bridge.cv2_to_imgmsg(cropped_img_rgb, encoding="passthrough")

        # Publish the ROS2 Image
        self.publisher_.publish(ros_frame)

    def publish_message(self):
        msg = Image()  # Create an Image message
        self.publisher_.publish(msg)
        self.get_logger().info('Published: Hello, ROS2!')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
