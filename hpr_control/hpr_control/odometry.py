import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import serial

ser = serial.Serial('/dev/ttyACM0', 115200)


class TwistSubscriber(Node):

    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.ser = serial.Serial('/dev/ttyACM0', 115200)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_cb,
            1)
        self.subscription  # prevent unused variable warning

    def cmd_vel_cb(self, msg):
        target_lin = msg.linear.x
        target_ang = msg.angular.z
        target_vel_str = f"{target_lin},{target_ang}\n"
        print(target_vel_str)
        self.ser.write(bytes(target_vel_str.encode('utf-8')))


def main(args=None):
    rclpy.init(args=args)

    twist_subscriber = TwistSubscriber()

    rclpy.spin(twist_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    twist_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
