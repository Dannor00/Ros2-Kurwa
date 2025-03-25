import os
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64


class JointSimulator:
    def __init__(self, K, T, noise):
        # Initialize variables
        self.angle = 0.0
        self.angular_velocity = 0.0
        self.voltage = 0.0
        self.noise = noise  # Noise value from parameter

        # Constants
        self.K = K  # Gain (rad)
        self.T = T  # Time constant (seconds)
        self.dt = 0.01  # Time step (seconds)

    def update(self):
        # Update angular velocity and angle based on the formula
        self.angular_velocity += (self.K * self.voltage - self.angular_velocity) * (self.dt / self.T)
        self.angle += self.angular_velocity * self.dt

        # Add random noise if applicable
        self.angle += self.noise


class JointSimulatorNode(Node):
    def __init__(self):
        super().__init__('joint_simulator_node')

        # Declare parameters
        self.declare_parameter('K', 230.0)
        self.declare_parameter('T', 0.15)
        self.declare_parameter('noise', 0.0)

        # Retrieve parameters
        self.K = self.get_parameter('K').get_parameter_value().double_value
        self.T = self.get_parameter('T').get_parameter_value().double_value
        self.noise = self.get_parameter('noise').get_parameter_value().double_value

        # Create an instance of the JointSimulator
        self.simulator = JointSimulator(self.K, self.T, self.noise)

        # Create publisher for measured angle
        self.publisher = self.create_publisher(Float64, '/measured_angle', 10)

        # Create subscriber for input voltage
        self.subscriber = self.create_subscription(
            Float64,
            '/voltage',
            self.voltage_listener,
            10
        )

        # Timer to update the simulator and publish angle
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Register parameter update callback
        self.add_on_set_parameters_callback(self.parameter_callback)

    def timer_callback(self):
        # Update the simulator and publish the angle
        self.simulator.update()
        angle_msg = Float64()
        angle_msg.data = self.simulator.angle
        self.publisher.publish(angle_msg)

        # Clear console and print updated values
        os.system('clear')  # Use 'cls' for Windows
        print(f"\033[1;34mJoint Simulator Node Running...\033[0m")
        print(f"Voltage: {self.simulator.voltage:.3f} V")
        print(f"Angular Velocity: {self.simulator.angular_velocity:.3f} rad/s")
        print(f"Angle: {self.simulator.angle:.3f} rad\n")

    def voltage_listener(self, msg):
        # Update voltage in the simulator
        self.simulator.voltage = msg.data

    def parameter_callback(self, params):
        """Callback to handle parameter updates."""
        for param in params:
            if param.name == 'K':
                self.K = param.value
                self.simulator.K = self.K  # Update simulator
                self.get_logger().info(f'K was set: {self.simulator.K}')
            elif param.name == 'T':
                self.T = param.value
                self.simulator.T = self.T  # Update simulator
                self.get_logger().info(f'T was set: {self.simulator.T}')
            elif param.name == 'noise':
                self.noise = param.value
                self.simulator.noise = self.noise  # Update simulator
                self.get_logger().info(f'Noise was set: {self.simulator.noise}')
        
        return SetParametersResult(successful=True, reason='Parameters updated successfully')


def main(args=None):
    rclpy.init(args=args)
    node = JointSimulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

