import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class PriusCmdVelNode(Node):
    def __init__(self):
        super().__init__('prius_cmd_vel_mirror')
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.publisher = self.create_publisher(
            Twist, '/prius/cmd_vel', 10
        )

    def cmd_vel_callback(self, msg):
        # Process the received 'msg' and publish it to '/prius/cmd_vel'
        self.publisher.publish(msg)

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