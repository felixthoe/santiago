import numpy as np
import sympy as sp

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration  # nicht zwingend nötig, aber sauber


class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('trajectory_publisher')

        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        self.timer_period = 0.1
        self.timer = self.create_timer(
            self.timer_period,
            self.controller_publisher_callback
        )

        self.controller_call_counter = 0
        self.start_time = None   # wird im ersten Callback gesetzt

        # Time targets
        self.t_01_target = 25.0
        self.t_p_target = 2.0
        self.t_12_target = 15.0

        self.t_total = self.t_01_target + self.t_p_target + self.t_12_target

        # Symbolic time variable
        t = sp.symbols('t')

        # Quintic profile segment 1
        sigma_01 = (
            10 * (t / self.t_01_target)**3
            - 15 * (t / self.t_01_target)**4
            + 6 * (t / self.t_01_target)**5
        )
        dsigma_01 = sp.diff(sigma_01, t)

        # Quintic profile segment 2
        sigma_12 = (
            10 * (t / self.t_12_target)**3
            - 15 * (t / self.t_12_target)**4
            + 6 * (t / self.t_12_target)**5
        )
        dsigma_12 = sp.diff(sigma_12, t)

        # Numeric functions
        self.sigma_01_func = sp.lambdify(t, sigma_01, 'numpy')
        self.dsigma_01_func = sp.lambdify(t, dsigma_01, 'numpy')
        self.sigma_12_func = sp.lambdify(t, sigma_12, 'numpy')
        self.dsigma_12_func = sp.lambdify(t, dsigma_12, 'numpy')

        # Reference target points
        self.q_0_ref = np.array([0.0, 0.0, 0.0])                    # [0°, 0 m, 0°]
        self.q_1_ref = np.array([0.610865, 0.1, 0.698132])        # [-35°, 0.1 m, -40°]
        self.q_2_ref = np.array([1.047198, 0.1, 0.820305])        # [-60°, 0.1 m, -47°]
        
        self.joint_names = [
            'upper_cylinder_upper_motor_joint',
            'column_outside_column_inside_joint',
            'upper_motor_box_arm_joint'
        ]

    def target_calculator(self, t):
        """
        Calculates q_ref and dq_ref for the current time t (seconds from start).
        """
        # Segment 1: q0 -> q1
        if 0.0 <= t <= self.t_01_target:
            tau = t
            sigma = self.sigma_01_func(tau)
            dsigma = self.dsigma_01_func(tau)
            q_ref = self.q_0_ref + sigma * (self.q_1_ref - self.q_0_ref)
            dq_ref = dsigma * (self.q_1_ref - self.q_0_ref)

        # Pause: hold q1
        elif self.t_01_target < t <= self.t_01_target + self.t_p_target:
            q_ref = self.q_1_ref.copy()
            dq_ref = np.zeros(3)

        # Segment 2: q1 -> q2
        elif self.t_01_target + self.t_p_target < t <= self.t_total:
            tau = t - self.t_01_target - self.t_p_target
            sigma = self.sigma_12_func(tau)
            dsigma = self.dsigma_12_func(tau)
            q_ref = self.q_1_ref + sigma * (self.q_2_ref - self.q_1_ref)
            dq_ref = dsigma * (self.q_2_ref - self.q_1_ref)

        # After trajectory end: hold q2
        else:
            q_ref = self.q_2_ref.copy()
            dq_ref = np.zeros(3)

        return q_ref, dq_ref

    def controller_publisher_callback(self):
        # Set start time on first call
        if self.start_time is None:
            self.start_time = self.get_clock().now()
        
        self.get_logger().info("publishing...")

        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        msg.header.stamp = self.start_time.to_msg()   

        t_global = self.controller_call_counter * self.timer_period

        N = 2
        dt = self.timer_period

        for i in range(N):
            t_i = t_global + i * dt         
            q_ref, dq_ref = self.target_calculator(t_i)

            point = JointTrajectoryPoint()
            point.positions = q_ref.tolist()
            point.velocities = dq_ref.tolist()

           
            time_from_start = t_i
            point.time_from_start.sec = int(time_from_start)
            point.time_from_start.nanosec = int((time_from_start % 1.0) * 1e9)

            msg.points.append(point)

        self.publisher_.publish(msg)
        self.controller_call_counter += 1

     
        if t_global >= self.t_total:
            self.get_logger().info("Trajectory finished, cancelling timer.")
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    trajectory_publisher = TrajectoryPublisher()
    rclpy.spin(trajectory_publisher)
    trajectory_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()