import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class PriusCmdVelNode(Node):
    def __init__(self):
        super().__init__('prius_cmd_vel_traffic_light')
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.status_subscription = self.create_subscription(
            String, '/status_message', self.status_callback, 10
        )
        self.publisher = self.create_publisher(
            Twist, '/prius/cmd_vel', 10
        )
        self.flag = 0
        self.cmd_vel_msg = Twist()

    def cmd_vel_callback(self, msg):
        # Store the cmd_vel message for future use
        self.cmd_vel_msg = msg

    def status_callback(self, msg):
        # Process the status message and set prius/cmd_vel components accordingly
        if msg.data == "1":  # RED
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.linear.y = 0.0
            self.cmd_vel_msg.linear.z = 0.0
            self.cmd_vel_msg.angular.x = 0.0
            self.cmd_vel_msg.angular.y = 0.0
            self.cmd_vel_msg.angular.z = 0.0
        elif msg.data == "2":  # YELLOW
            self.cmd_vel_msg.linear.x *= 0.5
            self.cmd_vel_msg.linear.y *= 0.5
            self.cmd_vel_msg.angular.z *= 0.5
        # For GREEN and DEFAULT status, keep the cmd_vel_msg as it is

        # Publish the modified cmd_vel message to '/prius/cmd_vel'
        self.publisher.publish(self.cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PriusCmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
