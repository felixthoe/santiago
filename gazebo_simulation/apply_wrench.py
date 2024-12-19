import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import ApplyLinkWrench
from builtin_interfaces.msg import Duration

class ApplyWrenchNode(Node):
    def __init__(self):
        super().__init__('apply_wrench_node')
        self.client = self.create_client(ApplyLinkWrench, '/apply_link_wrench')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /apply_link_wrench not available, waiting...')
        self.timer = self.create_timer(1.0, self.send_request)

    def send_request(self):
        current_sim_time = self.get_clock().now().to_msg()
        request = ApplyLinkWrench.Request()
        request.link_name = 'robot::arm_first_link' # Gazebo sees the whole kinematic chain of the three arm links as one link (called robot::arm_first_link) since they are connected by fixed joints
        request.reference_frame = 'robot::arm_first_link'  # Can be changed, if you want to specify the wrench vector and reference point in the world frame (then it would be robot::map)
        request.reference_point.x = -0.34 + 0.02  # 0.34 is the length of the second arm link, 0.02 is the radius of the third arm link
        request.reference_point.y = 0.0
        request.reference_point.z = 0.161204 + 2*0.02 + 0.757988/2  # 0.161204 is the length of the first arm link, 2*0.02 is the distance between the arm links (because of the second arm link), 0.757988 is the length of the third arm link
        request.wrench.force.x = -1e2  # Replace with your desired force
        request.wrench.force.y = 0.0
        request.wrench.force.z = 0.0
        request.wrench.torque.x = 0.0  # Replace with your desired torque
        request.wrench.torque.y = 0.0
        request.wrench.torque.z = 0.0
        request.duration.sec = current_sim_time.sec + 1  # Duration for which the force/torque is applied

        self.get_logger().info(f'Sending request: {request}')
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error('Service call to gazebos /apply_link_wrench failed: Response was unsuccessful')
            else:
                self.get_logger().info(f'Wrench applied succesfully')
        except Exception as e:
            self.get_logger().error(f'Service call to gazebos /apply_link_wrench faied: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ApplyWrenchNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()