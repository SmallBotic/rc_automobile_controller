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
        print("Automobile Controller Node Started")

    def send_cmd_vel(self, linear_x, angular_z):
        """
        Send cmd_vel to the automobile.
        `linear_x:` Linear velocity in x axis (backward/forward)
        `angular_z:` Angular velocity in z axis (left/right)
        `return:` None
        """
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub_.publish(msg)

    def imu_callback(self, imu_data: String):
        # print("IMU:", imu_data)
        self.imu_data = imu_data.data
        self.print_data()
        # acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z
        # imu = imu_data.split(",")
        # accel = [float(i) for i in imu[:3]]
        # gyro = [float(i) for i in imu[3:6]]
        # mag = [float(i) for i in imu[6:]]

        # print(
        #     "AX:",
        #     accel[0],
        #     "AY:",
        #     accel[1],
        #     "AZ:",
        #     accel[2],
        #     "GX:",
        #     gyro[0],
        #     "GY:",
        #     gyro[1],
        #     "GZ:",
        #     gyro[2],
        #     "MX:",
        #     mag[0],
        #     "MY:",
        #     mag[1],
        #     "MZ:",
        #     mag[2],
        # )

    def distance_callback(self, distance: Int32):
        # print("Distance:", distance)
        self.distance_data = distance.data
        self.print_data()

    def print_data(self):
        print(" " * 100, end="\r")
        print(
            f"IMU: [green][bold]{self.imu_data}[/bold][/green] [white]Distance:[/white] [red][bold]{self.distance_data}[/bold][/red]",
            end="\r",
        )


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
