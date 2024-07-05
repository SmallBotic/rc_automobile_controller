from rich import print

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String

from .config import (
    CMD_VEL_PUBLISH_TOPIC,
    IMU_SUBSCRIPTION_TOPIC,
    DISTANCE_SUBSCRIPTION_TOPIC,
)

from random import random, choice


class AutomobileController(Node):
    def __init__(self):
        super().__init__("rc_automobile_controller")
        self.cmd_vel_pub_ = self.create_publisher(Twist, CMD_VEL_PUBLISH_TOPIC, 10)
        self.imu_sub_ = self.create_subscription(
            String, IMU_SUBSCRIPTION_TOPIC, self.imu_callback, 10
        )
        self.distance_sub_ = self.create_subscription(
            Int32, DISTANCE_SUBSCRIPTION_TOPIC, self.distance_callback, 10
        )

        self.imu_data = None
        self.distance_data = None
        self.cmd_published = False
        print("Automobile Controller Node Started")
        self.teleop_publisher = self.create_publisher(Twist, CMD_VEL_PUBLISH_TOPIC, 10)
        # self.send_cmd_vel()
        self.create_timer(0.1, self.send_cmd_vel_handler)

    def send_cmd_vel_handler(self):
        choices = [1, -1]
        self.send_cmd_vel(random() * choice(choices), random() * choice(choices))
        self.cmd_published = True

    def send_cmd_vel(self, linear_x, angular_z):
        """
        Send cmd_vel to the automobile.
        `linear_x:` Linear velocity in x axis (backward/forward)
        `angular_z:` Angular velocity in z axis (left/right)
        `returns:` None
        """
        try:
            msg = Twist()
            msg.linear.x = linear_x
            msg.angular.z = angular_z
            self.cmd_vel_pub_.publish(msg)
            # self.send_control()
        except TypeError as e:
            print(e)

    def imu_callback(self, imu_data: String):
        # print("IMU:", imu_data)
        self.imu_data = imu_data.data
        self.print_data()
        # acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z
        # imu = imu_data.split(",")
        # accel = [float(i) for i in imu[:3]]
        # gyro = [float(i) for i in imu[3:6]]
        # mag = [float(i) for i in imu[6:]]

    def distance_callback(self, distance: Int32):
        # print("Distance:", distance)
        self.distance_data = distance.data
        self.print_data()

    def print_data(self):
        print(" " * 100, end="\r")
        print(
            f"IMU: [green][bold]{self.imu_data}[/bold][/green] [white]Distance:[/white] [red][bold]{self.distance_data}[/bold][/red] [green][bold]{self.cmd_published}[/bold][/green]",
            end="\r",
        )
        self.cmd_published = False


def main(args=None):
    try:
        rclpy.init(args=args)

        node = AutomobileController()

        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("\nBye")
    pass


if __name__ == "__main__":
    main()
