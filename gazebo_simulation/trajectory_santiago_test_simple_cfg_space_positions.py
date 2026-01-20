import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryPublisherSantiago(Node):
    def __init__(self, node_name, pub_period, pub_max_iterations):
        super().__init__(node_name)

        # Attributes
        self.pub_period = float(pub_period)                 # seconds
        self.pub_max_iterations = int(pub_max_iterations)

        # Publisher
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Phase boundaries
        self.pub_max_iterations_one_third = int(self.pub_max_iterations / 3)
        self.pub_max_iterations_two_thirds = int(2 * self.pub_max_iterations / 3)

        # Iterator
        self.i = 0

        # Timer
        self.timer = self.create_timer(self.pub_period, self.timer_callback)

    def timer_callback(self):
        # Build a fresh message each tick (prevents unbounded growth)
        msg = JointTrajectory()
        msg.joint_names = ['base_rotation_joint', 'arm_height_joint', 'arm_rotation_joint']

        # Compute positions
        if self.i < self.pub_max_iterations_one_third:
            a = self.i / self.pub_max_iterations_one_third
            positions = [
                a * math.pi * 0.0,
                0 + a * -0.25,
                a * math.pi * -0.5
            ]

        elif self.i < self.pub_max_iterations_two_thirds:
            a = (self.i - self.pub_max_iterations_one_third) / (
                self.pub_max_iterations_two_thirds - self.pub_max_iterations_one_third
            )
            positions = [
                math.pi * 0.5 * (1 - a),
                -0.1 - a * 0.2,
                math.pi * 0.5 * (1 - a)
            ]

        elif self.i <= self.pub_max_iterations:
            a = (self.i - self.pub_max_iterations_two_thirds) / (
                self.pub_max_iterations - self.pub_max_iterations_two_thirds
            )
            positions = [
                a * math.pi * 0.5,
                -0.3 - a * -0.3,
                a * math.pi * 0.5
            ]

        else:
            positions = [0.0, 0.0, 0.0]


        if self.i <= self.pub_max_iterations:
            # Create a fresh point
            point = JointTrajectoryPoint()
            point.positions = [float(x) for x in positions]

            # time_from_start (relative time within this trajectory message)
            t = self.i * self.pub_period
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)

            msg.points.append(point)
            self.publisher.publish(msg)

            self.i += 1

def main(args=None):
    print("TrajectoryPublisher node started")
    rclpy.init(args=args)
    node = TrajectoryPublisherSantiago("santiago_trajectory_publisher", 0.01, 2000)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
