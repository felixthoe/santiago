import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import ApplyJointEffort
from sensor_msgs.msg import JointState

EFFORTS = {
    'upper_cylinder_upper_motor_joint':   50.0,   
    'column_outside_column_inside_joint': 1600.0, 
    'upper_motor_box_arm_joint':          -50.0,   
}

class TestEffortNode(Node):
    def __init__(self):
        super().__init__('test_effort_node')

        self.client = self.create_client(ApplyJointEffort, '/apply_joint_effort')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /apply_joint_effort...')

        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self.js_callback, 10)

        self.create_timer(0.1, self.apply_effort)

    def js_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in EFFORTS:
                if i < len(msg.position) and i < len(msg.velocity):
                    self.get_logger().info(
                        f'{name}: pos={msg.position[i]:.4f}  vel={msg.velocity[i]:.4f}',
                        throttle_duration_sec=0.5)

    def apply_effort(self):
        for joint_name, effort in EFFORTS.items():
            req = ApplyJointEffort.Request()
            req.joint_name = joint_name
            req.effort = effort
            req.start_time.sec = 0
            req.start_time.nanosec = 0
            req.duration.sec = 1
            req.duration.nanosec = 0
            future = self.client.call_async(req)
            future.add_done_callback(
                lambda f, n=joint_name: self.get_logger().info(
                    f'{n}: success={f.result().success} message={f.result().status_message}',
            )
        )

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TestEffortNode())

if __name__ == '__main__':
    main()