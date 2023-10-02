import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyToTwist(Node):

    def __init__(self):
        super().__init__('joy_to_twist')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',  # Replace with your Joy topic name
            self.joy_callback,
            10)
        self.publisher = self.create_publisher(
            Twist,
            'piRobot/cmd_vel',  # Replace with your Twist topic name
            10)

    def joy_callback(self, msg):
        twist_msg = Twist()

        twist_msg.linear.x = msg.axes[1] 
        twist_msg.linear.y = msg.axes[0] 
        twist_msg.angular.z = msg.axes[3] 
        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    joy_to_twist_node = JoyToTwist()
    rclpy.spin(joy_to_twist_node)
    joy_to_twist_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
