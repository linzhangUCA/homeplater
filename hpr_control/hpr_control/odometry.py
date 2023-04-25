import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import serial

ser = serial.Serial('/dev/ttyACM0', 115200)


class HPRInterface(Node):
    """
    A ROS interface for the HomeplateRobot
    """

    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.ser = serial.Serial('/dev/ttyACM0', 115200)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_cb,
            1)
        self.cmd_vel_sub  # prevent unused variable warning
        self.vel_pub = self.create_publisher(Twist, 'hpr_vel', 1)
        self.vel_pub_timer = self.create_timer(0.02, self.vel_pub_cb)
        # variable
        self.lin_x = 0.
        self.ang_z = 0.

    def cmd_vel_cb(self, msg):
        target_lin = msg.linear.x
        target_ang = msg.angular.z
        target_vel_str = f"{target_lin},{target_ang}\n"
        # print(target_vel_str)
        self.ser.write(bytes(target_vel_str.encode('utf-8')))

    def vel_pub_cb(self):
        vel_msg = Twist()
        if self.ser.inWaiting() > 0:
            data_line = self.ser.readline().decode('utf-8').rstrip()
            vel_list = data_line.split(',')
            self.lin_x = float(vel_list[0])
            self.ang_z = float(vel_list[1])
            vel_msg.linear.x = self.lin_x
            vel_msg.angular.z = self.ang_z
        self.vel_pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)

    hpr_interface = HPRInterface()

    rclpy.spin(hpr_interface)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hpr_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
