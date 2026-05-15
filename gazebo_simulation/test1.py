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
            a = self.i / max(1, (self.pub_max_iterations_one_third - 1))
            positions = [
                0.5 * math.pi + a * (-0.5 * math.pi),    # J1: pi/2 -> 0
                0.0 + a * (-0.4),                        # J2: 0 -> -0.4
                0.0 + a * (-0.5 * math.pi)               # J3: 0 -> -pi/2
            ]

        elif self.i < self.pub_max_iterations_two_thirds:
            phase_len = self.pub_max_iterations_two_thirds - self.pub_max_iterations_one_third
            a = (self.i - self.pub_max_iterations_one_third) / max(1, (phase_len - 1))
            positions = [
                0.0,                                    # J1: hold at 0
                -0.4 + a * (0.4),                       # J2: -0.4 -> 0
                -0.5 * math.pi + a * (-0.5 * math.pi)   # J3: -pi/2 -> -pi
            ]

        elif self.i <= self.pub_max_iterations:
            phase_len = self.pub_max_iterations - self.pub_max_iterations_two_thirds
            a = (self.i - self.pub_max_iterations_two_thirds) / max(1, phase_len)
            positions = [
                a * (0.5 * math.pi),                    # J1: 0 -> pi/2
                0.0,                                    # J2: hold at 0
                -1.0 * math.pi                          # J3: hold at -pi
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