import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np

class MyNode(Node):
    def __init__(self):
        super().__init__('traffic_light_detect')
        self.publisher_image = self.create_publisher(Image, '/output_opencv', 10)
        self.publisher_status = self.create_publisher(String, '/status_message', 10)

        self.subscription = self.create_subscription(Image, '/traffic_light_cropped', self.callback, 10)

    def filter_circular_blobs(self, image):
        # Convert the image to grayscale
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

        # Apply HoughCircles to detect circular blobs
        circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, dp=1, minDist=5, param1=200, param2=1, minRadius=1, maxRadius=50)

        # Create a mask to filter out non-circular blobs
        mask = np.zeros_like(gray)
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                cv.circle(mask, (x, y), r, (255), -1)

        # Apply the mask to the original image
        filtered_image = cv.bitwise_and(image, image, mask=mask)

        return filtered_image

    def callback(self, msg):
        self.get_logger().info('Received an image message')

        # Convert ROS2 Image message to OpenCV image
        cv_bridge = CvBridge()
        frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Process the received message here
        gauss = cv.GaussianBlur(frame, (5, 5), 0)
        hsv_image = cv.cvtColor(gauss, cv.COLOR_BGR2HSV)

        # After finding the appropriate HSV values from the hsv_filter node, make the masks
        lower_green = (45, 160, 210)
        upper_green = (70, 255, 255)
        mask_green = cv.inRange(hsv_image, lower_green, upper_green)

        lower_red = (0, 160, 210)
        upper_red = (20, 255, 255)
        mask_red = cv.inRange(hsv_image, lower_red, upper_red)

        lower_yellow = (25, 160, 210)
        upper_yellow = (50, 255, 255)
        mask_yellow = cv.inRange(hsv_image, lower_yellow, upper_yellow)

        filtered_green = cv.bitwise_and(hsv_image, hsv_image, mask=mask_green)
        filtered_yellow = cv.bitwise_and(hsv_image, hsv_image, mask=mask_yellow)
        filtered_red = cv.bitwise_and(hsv_image, hsv_image, mask=mask_red)

        # Apply erosion and dilation (opening) to remove noise
        kernel = np.ones((5, 5), np.uint8)
        filtered_green = cv.morphologyEx(filtered_green, cv.MORPH_OPEN, kernel)
        filtered_yellow = cv.morphologyEx(filtered_yellow, cv.MORPH_OPEN, kernel)
        filtered_red = cv.morphologyEx(filtered_red, cv.MORPH_OPEN, kernel)

        # Filter circular blobs in the final image
        filtered_green_circles = self.filter_circular_blobs(filtered_green)
        filtered_yellow_circles = self.filter_circular_blobs(filtered_yellow)
        filtered_red_circles = self.filter_circular_blobs(filtered_red)

        # Determine the final image based on the predominant color
        flag = 4
        final_img = filtered_red_circles
        if cv.countNonZero(mask_green) > cv.countNonZero(mask_yellow) and cv.countNonZero(mask_green) > cv.countNonZero(mask_red):
            final_img = filtered_green_circles
            flag = 3
        elif cv.countNonZero(mask_yellow) > cv.countNonZero(mask_red):
            final_img = filtered_yellow_circles
            flag = 2
        elif cv.countNonZero(mask_red) > 1:
            final_img = filtered_red_circles
            flag = 1

        # Convert OpenCV image to ROS2 Image
        bridge = CvBridge()
        ros_frame = bridge.cv2_to_imgmsg(final_img, encoding="passthrough")

        # Publish the ROS2 Image
        self.publisher_image.publish(ros_frame)

        # Publish the status message
        status_msg = String()
        if flag == 3:
            status_msg.data = "3"
            self.get_logger().info('GREEN')
        elif flag == 2:
            status_msg.data = "2"
            self.get_logger().info('YELLOW')
        elif flag == 1:
            status_msg.data = "1"
            self.get_logger().info('RED')
        else:
            status_msg.data = "4"
            self.get_logger().info('NONE')
        self.publisher_status.publish(status_msg)

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
