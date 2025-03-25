import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from pid_controller_msgs.srv import SetReference
import math

class PidControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # PID-parametere
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)

        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value

        self.get_logger().info(f"✅ Bruker PID-parametre: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}")

        # Interne variabler
        self.reference = 0.0
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = self.get_clock().now().nanoseconds / 1e9
        self.current_angle = 0.0

        # Subscriber for vinkel
        self.subscriber = self.create_subscription(
            Float64,
            '/measured_angle',
            self.measured_callback,
            10
        )

        # Publisher for spenning
        self.voltage_publisher = self.create_publisher(Float64, '/voltage', 10)

        # Service for å sette referanse
        self.srv = self.create_service(SetReference, 'set_reference', self.set_reference_callback)

        # PID-kontroll-loop
        self.timer = self.create_timer(0.01, self.control_loop)

    def set_reference_callback(self, request, response):
        if -math.pi <= request.request <= math.pi:
            self.reference = request.request
            self.get_logger().info(f"✅ Ny referanseverdi satt til: {self.reference:.2f} rad")
            response.success = True
        else:
            self.get_logger().warn(f"❌ Ugyldig referanseverdi: {request.request:.2f}")
            response.success = False
        return response

    def measured_callback(self, msg):
        self.current_angle = msg.data

    def control_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last_time
        if dt <= 0.0:
            return

        error = self.reference - self.current_angle
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        voltage = self.kp * error + self.ki * self.integral + self.kd * derivative

        voltage_msg = Float64()
        voltage_msg.data = voltage
        self.voltage_publisher.publish(voltage_msg)

        self.prev_error = error
        self.last_time = now

def main(args=None):
    rclpy.init(args=args)
    node = PidControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

