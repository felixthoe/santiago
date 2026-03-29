import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('trajectory_publisher') # Node Name
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.timer_period = 10  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        msg = JointTrajectory()
        msg.joint_names = ['upper_cylinder_upper_motor_joint', 'column_outside_column_inside_joint', 'upper_motor_box_arm_joint']
        
        for i in range(self.timer_period+1):
            point = JointTrajectoryPoint()

            if i < 5:
                point.positions = [float(x) for x in [-(i/4)*np.pi*0.388, -0.3 + (i/4)*0.1, -(i/4*np.pi)/4]]
            if i > 4 and i < 10:
                point.positions = [float(x) for x in [-np.pi*0.388 + ((i-5)/4)*np.pi*0.388, -0.2 - ((i-5)/4)*0.1, -np.pi/4 + ((i-5)/4)*np.pi/4]]
            if i == 10:
                point.positions = [float(x) for x in [0, -0.3, 0]]

            point.time_from_start.sec = i
            msg.points.append(point)
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    trajectory_publisher = TrajectoryPublisher()
    rclpy.spin(trajectory_publisher)
    trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()