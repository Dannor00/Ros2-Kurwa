import os
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64


class JointSimulator:
    def __init__(self, K, T, noise):
        self.angle = 0.0
        self.angular_velocity = 0.0
        self.voltage = 0.0
        self.noise = noise

        self.K = K
        self.T = T
        self.dt = 0.01  # Fast tidssteg

    def update(self):
        # Oppdater hastighet og vinkel basert på input og modell
        self.angular_velocity += (self.K * self.voltage - self.angular_velocity) * (self.dt / self.T)
        self.angle += self.angular_velocity * self.dt

        # Legg til støy hvis satt
        self.angle += self.noise


class JointSimulatorNode(Node):
    def __init__(self):
        super().__init__('joint_simulator_node')

        # Deklarer parametere
        self.declare_parameter('K', 230.0)
        self.declare_parameter('T', 0.15)
        self.declare_parameter('noise', 0.0)

        # Hent parametere
        self.K = self.get_parameter('K').get_parameter_value().double_value
        self.T = self.get_parameter('T').get_parameter_value().double_value
        self.noise = self.get_parameter('noise').get_parameter_value().double_value

        # Initialiser simulator
        self.simulator = JointSimulator(self.K, self.T, self.noise)

        # Publisher
        self.publisher = self.create_publisher(Float64, '/measured_angle', 10)

        # Subscriber
        self.subscriber = self.create_subscription(
            Float64,
            '/voltage',
            self.voltage_listener,
            10
        )

        # Timer for å oppdatere simulator og sende måling
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Tillat dynamisk oppdatering av parametere
        self.add_on_set_parameters_callback(self.parameter_callback)

    def timer_callback(self):
        self.simulator.update()
        angle_msg = Float64()
        angle_msg.data = self.simulator.angle
        self.publisher.publish(angle_msg)

        # Konsollutskrift
        os.system('clear')  # Bruk 'cls' hvis du er på Windows
        print(f"\033[1;34mJoint Simulator Node Running...\033[0m")
        print(f"Voltage:           {self.simulator.voltage:.3f} V")
        print(f"Angular Velocity:  {self.simulator.angular_velocity:.3f} rad/s")
        print(f"Angle:             {self.simulator.angle:.3f} rad\n")

    def voltage_listener(self, msg):
        self.simulator.voltage = msg.data

    def parameter_callback(self, params):
        """Oppdater K, T eller noise. Nullstill simulator om K eller T endres."""
        reset = False
        for param in params:
            if param.name == 'K':
                self.K = param.value
                self.simulator.K = self.K
                self.get_logger().info(f'🔧 K satt til: {self.K}')
                reset = True
            elif param.name == 'T':
                self.T = param.value
                self.simulator.T = self.T
                self.get_logger().info(f'🔧 T satt til: {self.T}')
                reset = True
            elif param.name == 'noise':
                self.noise = param.value
                self.simulator.noise = self.noise
                self.get_logger().info(f'🔧 Noise satt til: {self.noise}')

        if reset:
            self.simulator.angle = 0.0
            self.simulator.angular_velocity = 0.0
            self.get_logger().info('♻️  Simulator nullstilt etter parameterendring.')

        return SetParametersResult(successful=True, reason='Parametre oppdatert')

def main(args=None):
    rclpy.init(args=args)
    node = JointSimulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

