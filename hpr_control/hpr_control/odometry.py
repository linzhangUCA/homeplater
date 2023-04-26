import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_about_axis

import serial

from math import sin, cos, pi


class HPRInterface(Node):
    """
    A ROS interface for the HomeplateRobot
    """

    def __init__(self):
        super().__init__('hpr_interface')
        self.ser = serial.Serial('/dev/ttyACM0', 115200)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_cb,
            1
        )
        self.trans_broad = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 1)
        self.odom_pub_timer = self.create_timer(0.02, self.odom_pub_cb)
        # variables
        self.lin_vel = 0.  # in base_link frame
        self.ang_vel = 0.
        self.x = 0.  # in odom frame
        self.y = 0.
        self.th = 0.
        self.prev_ts = self.get_clock().now()
        self.curr_ts = self.get_clock().now()
        # constants
        self.GROUND_CLEARANCE = 0.0375  # wheel radius

    def cmd_vel_cb(self, msg):
        target_lin = msg.linear.x
        target_ang = msg.angular.z
        target_vel_str = f"{target_lin},{target_ang}\n"
        self.ser.write(bytes(target_vel_str.encode('utf-8')))

    def odom_pub_cb(self):
        if self.ser.inWaiting() > 0:
            data_line = self.ser.readline().decode('utf-8').rstrip()
            vel_list = data_line.split(',')
            self.lin_vel = float(vel_list[0])
            self.ang_vel = float(vel_list[1])
        self.curr_ts = self.get_clock().now()
        dt = (self.curr_ts - self.prev_ts).nanoseconds * 1e-9
        dx = self.lin_vel * cos(self.th) * dt
        dy = self.lin_vel * sin(self.th) * dt
        dth = self.ang_vel * dt
        self.x += dx
        self.y += dy
        self.th += dth
        if self.th > pi:
            self.th -= 2 * pi
        elif self.th < -pi:
            self.th += 2 * pi
        quat = quaternion_about_axis(self.th, (0, 0, 1))
        self.prev_ts = self.curr_ts
        # publish transform
        trans_msg = TransformStamped()
        trans_msg.header.stamp = self.curr_ts.to_msg()
        trans_msg.header.frame_id = "odom"
        trans_msg.child_frame_id = "base_link"
        trans_msg.transform.translation.x = self.x
        trans_msg.transform.translation.y = self.y
        trans_msg.transform.translation.z = self.GROUND_CLEARANCE
        trans_msg.transform.rotation.x = quat[0]
        trans_msg.transform.rotation.y = quat[1]
        trans_msg.transform.rotation.z = quat[2]
        trans_msg.transform.rotation.w = quat[3]
        self.trans_broad.sendTransform(trans_msg)
        # publish odom
        odom_msg = Odometry()
        odom_msg.header.stamp = self.curr_ts.to_msg()
        # odom_msg.header.stamp.sec = self.curr_ts.seconds_nanoseconds()[0]
        # odom_msg.header.stamp.nanosec = self.curr_ts.seconds_nanoseconds()[1]
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = self.GROUND_CLEARANCE
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        odom_msg.twist.twist.linear.x = self.lin_vel
        odom_msg.twist.twist.angular.z = self.ang_vel
        self.odom_pub.publish(odom_msg)


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
