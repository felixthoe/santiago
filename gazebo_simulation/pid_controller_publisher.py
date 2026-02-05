import numpy as np
from control_msgs.msg import MultiDOFCommand
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class PIDControllerPublisher(Node):

    def __init__(self):
        super().__init__('pid_controller_publisher') # Node name

        # Create publishers for each joint controller
        self.publisher = self.create_publisher(MultiDOFCommand, '/pid_controller/reference', 10)

        # Create a timer to periodically publish commands
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Helper variable used to create some test signals
        self.i = 0
        self.reverse = False

    def timer_callback(self):
        # Create message
        msg = MultiDOFCommand()
        msg.dof_names = ['base_rotation_joint', 'arm_height_joint', 'arm_rotation_joint']

        # just some test signals
        # determining the direction of the movement
        if self.reverse==False:
            self.i += 0.01
            if self.i==1 or self.i>1:
                self.reverse=True
        else:
            self.i -= 0.01
            if self.i==0 or self.i<0:
                self.reverse=False
        
        self.base_rotation_reference = (self.i*1.5-1)*np.pi     # between -pi and pi/2 radians
        self.arm_height_reference = -self.i*0.6                 # between -0.6 and 0 meters
        self.arm_rotation_reference = -self.i*np.pi/2           # between -pi/2 and 0 radians

        # Fill in the message
        msg.values = [self.base_rotation_reference, self.arm_height_reference, self.arm_rotation_reference]

        # Publish messages
        self.publisher.publish(msg)

        self.get_logger().info(f'Publishing: base_rotation={self.base_rotation_reference}, arm_height={self.arm_height_reference}, arm_rotation={self.arm_rotation_reference}')

def main(args=None):
    rclpy.init(args=args)
    pid_controller_publisher = PIDControllerPublisher()
    rclpy.spin(pid_controller_publisher)
    pid_controller_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()