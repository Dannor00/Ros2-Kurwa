import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # PID-parametre
        self.kp = 5.0
        self.ki = 0.0
        self.kd = 0.1
        self.dt = 0.01

        self.error_sum = 0.0
        self.prev_error = 0.0
        self.target_position = 0.0  # ønsket vinkel

        # Abonnerer på joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        # Abonnerer på /reference for å sette ny ønsket vinkel
        self.reference_sub = self.create_subscription(
            Float64,
            '/reference',
            self.reference_callback,
            10)

        # Publisher til velocity_controller
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/velocity_controller/command',
            10)

        self.get_logger().info("PID controller node started")

    def joint_state_callback(self, msg):
        if not msg.position:
            self.get_logger().warn("Joint state has no position data")
            return

        position = msg.position[0]
        error = self.target_position - position
        self.error_sum += error * self.dt
        d_error = (error - self.prev_error) / self.dt

        control = self.kp * error + self.ki * self.error_sum + self.kd * d_error
        self.prev_error = error

        command_msg = Float64MultiArray()
        command_msg.data = [control]
        self.publisher.publish(command_msg)

    def reference_callback(self, msg):
        self.target_position = msg.data
        self.get_logger().info(f"New target position set to: {self.target_position}")

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

