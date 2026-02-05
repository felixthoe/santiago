import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # You can replace 'String' with the message type you are subscribing to
from control_msgs.msg import JointTrajectoryControllerState

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')  # Node name
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState,              # Message type
            '/joint_trajectory_controller/controller_state',     # Topic name
            self.listener_callback,  # Callback function
            10                   # QoS profile (queue size)
        )
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        position = msg.reference.positions[1]
        self.get_logger().info(f'Received: {position}')

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2 Python client library
    subscriber = SimpleSubscriber()   
    rclpy.spin(subscriber)  # Keep the node alive to listen for messages   
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()