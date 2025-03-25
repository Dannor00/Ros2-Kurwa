import rclpy
from rclpy.node import Node
from pid_controller_msgs.srv import SetReference

class ReferenceInputNode(Node):
    def __init__(self):
        super().__init__('reference_input_node')
        self.cli = self.create_client(SetReference, 'set_reference')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Venter på set_reference-service...')
        self.get_logger().info("Service funnet. Klar for input.")

    def send_reference(self, value):
        req = SetReference.Request()
        req.request = value
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().success:
                print("✅ Referanse sendt!")
            else:
                print("❌ Ugyldig referanse! Må være mellom -π og π.")
        else:
            print("❌ Tjenestekall feilet!")

def main(args=None):
    rclpy.init(args=args)
    node = ReferenceInputNode()
    try:
        while rclpy.ok():
            user_input = input("🧠 Sett inn ny referanse (radians):\n> ")
            try:
                ref = float(user_input)
                node.send_reference(ref)
            except ValueError:
                print("❌ Ikke et gyldig tall.")
    except KeyboardInterrupt:
        print("\nAvslutter.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
