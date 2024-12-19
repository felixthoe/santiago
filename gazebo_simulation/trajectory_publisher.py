import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.timer_period = 3  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        msg = JointTrajectory()
        msg.joint_names = ['base_rotation_joint', 'arm_height_joint', 'arm_rotation_joint']
        
        for i in range(self.timer_period+1):
            point = JointTrajectoryPoint()
            point.positions = [float(x) for x in [(i/self.timer_period*np.pi*1.5)-np.pi, -(i/self.timer_period)*0.6, -i/self.timer_period*np.pi/2]]
            point.time_from_start.sec = i
            msg.points.append(point)
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    trajectory_publisher = TrajectoryPublisher()
    rclpy.spin(trajectory_publisher)
    trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()