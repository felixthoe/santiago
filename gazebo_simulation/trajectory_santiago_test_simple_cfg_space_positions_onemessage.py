import math
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TrajectoryPublisherSantiago(Node):
    def __init__(self, node_name: str, pub_period: float, pub_max_iterations: int):
        super().__init__(node_name)

        self.pub_period = float(pub_period)          # seconds between points
        self.pub_max_iterations = int(pub_max_iterations)

        self.publisher = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10,
        )

        # phase boundaries
        self.n1 = int(self.pub_max_iterations / 3)
        self.n2 = int(2 * self.pub_max_iterations / 3)

        # Publish once, shortly after startup (gives controller time to connect)
        self.timer = self.create_timer(0.2, self.publish_once)
        self._published = False

    def _positions_at_i(self, i: int):
        # Your piecewise trajectory (kept as close as possible to your code)
        if i < self.n1:
            a = i / self.n1 if self.n1 > 0 else 0.0
            return [
                a * math.pi * 0.5,
                -0.5 + a * -0.4,  # goes from -0.5 to -0.9
                a * math.pi,
            ]

        if i < self.n2:
            denom = (self.n2 - self.n1) if (self.n2 - self.n1) > 0 else 1.0
            a = (i - self.n1) / denom
            return [
                math.pi * 0.5 * (1 - a),
                -0.9 - a * -0.4,  # goes from -0.9 back to -0.5
                math.pi * (1 - a),
            ]

        if i <= self.pub_max_iterations:
            denom = (self.pub_max_iterations - self.n2) if (self.pub_max_iterations - self.n2) > 0 else 1.0
            a = (i - self.n2) / denom
            return [
                a * math.pi * 0.5,
                -0.5 - a * 0.4,   # goes from -0.5 to -0.9
                a * math.pi,
            ]

        return [0.0, 0.0, 0.0]

    def publish_once(self):
        if self._published:
            return

        msg = JointTrajectory()
        msg.joint_names = ["base_rotation_joint", "arm_height_joint", "arm_rotation_joint"]

        # IMPORTANT:
        # - time_from_start should start > 0 (many controllers dislike 0.0 exactly)
        # - points must be increasing in time
        for i in range(self.pub_max_iterations + 1):
            point = JointTrajectoryPoint()
            point.positions = [float(x) for x in self._positions_at_i(i)]

            t = (i + 1) * self.pub_period  # start at pub_period instead of 0.0
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)

            msg.points.append(point)

        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published trajectory with {len(msg.points)} points, duration ~{(self.pub_max_iterations + 1) * self.pub_period:.2f}s"
        )

        self._published = True

        # Exit soon after publishing
        self.create_timer(0.5, self._shutdown)

    def _shutdown(self):
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    # Example: period=1s, iterations=10 => ~11 seconds duration
    node = TrajectoryPublisherSantiago("santiago_trajectory_publisher", pub_period=1.0, pub_max_iterations=10)

    rclpy.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()
