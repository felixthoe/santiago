import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class PIDControllerPublisher(Node):

    def __init__(self):
        super().__init__('pid_controller_publisher')

        # Create publishers for each joint controller
        self.base_rotation_publisher = self.create_publisher(Float64, '/base_rotation_controller/commands', 10)
        self.arm_height_publisher = self.create_publisher(Float64, '/arm_height_controller/commands', 10)
        self.arm_rotation_publisher = self.create_publisher(Float64, '/arm_rotation_controller/commands', 10)

        # Create a timer to periodically publish commands
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize command values
        self.base_rotation_command = 0.0
        self.arm_height_command = 0.0
        self.arm_rotation_command = 0.0

    def timer_callback(self):
        # Update command values (example: simple oscillation)
        self.base_rotation_command += 0.1
        self.arm_height_command += 0.05
        self.arm_rotation_command += 0.1

        # Create messages
        base_rotation_msg = Float64()
        base_rotation_msg.data = self.base_rotation_command

        arm_height_msg = Float64()
        arm_height_msg.data = self.arm_height_command

        arm_rotation_msg = Float64()
        arm_rotation_msg.data = self.arm_rotation_command

        # Publish messages
        self.base_rotation_publisher.publish(base_rotation_msg)
        self.arm_height_publisher.publish(arm_height_msg)
        self.arm_rotation_publisher.publish(arm_rotation_msg)

        self.get_logger().info(f'Publishing: base_rotation={self.base_rotation_command}, arm_height={self.arm_height_command}, arm_rotation={self.arm_rotation_command}')

def main(args=None):
    rclpy.init(args=args)
    pid_controller_publisher = PIDControllerPublisher()
    rclpy.spin(pid_controller_publisher)
    pid_controller_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()