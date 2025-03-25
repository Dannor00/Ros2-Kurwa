import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class PIDController:
    def __init__(self, p: float, i: float, d: float, reference: float):
        self.p = p
        self.i = i
        self.d = d
        self.reference = reference
        self.voltage = 0.0

        self.prev_error = 0.0
        self.integral = 0.0

    def update(self, measured_value: float):
        # Beregn feil
        error = self.reference - measured_value
        
        # Beregn integral og derivat
        self.integral += error
        derivative = error - self.prev_error

        # Beregn spenning (pådrag)
        self.voltage = self.p * error + self.i * self.integral + self.d * derivative

        # Oppdater forrige feil
        self.prev_error = error

        return self.voltage

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # Initialiser PIDController med tilfeldige verdier
        self.pid_controller = PIDController(p=1.0, i=0.1, d=0.01, reference=10.0)

        # Publisher for voltage
        self.publisher_ = self.create_publisher(Float64, '/voltage', 10)

        # Subscriber for measured_angle
        self.subscription = self.create_subscription(
            Float64,
            '/measured_angle',
            self.measurement_listener,
            10
        )
        self.get_logger().info("PIDControllerNode started.")

    def measurement_listener(self, msg: Float64):
        # Oppdater PID-kontrolleren med mottatt verdi
        voltage = self.pid_controller.update(msg.data)

        # Publiser oppdatert spenning
        self.publish_voltage(voltage)

    def publish_voltage(self, voltage: float):
        msg = Float64()
        msg.data = voltage
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published voltage: {voltage}")

def main(args=None):
    rclpy.init(args=args)

    pid_controller_node = PIDControllerNode()

    try:
        rclpy.spin(pid_controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        pid_controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

