import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger 

class QrCodeClient(Node):
    def __init__(self):
        super().__init__('qr_code_client')
        self.cli = self.create_client(Trigger, 'start_stop_qr_detection') # Client for your service
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Trigger.Request()

    def send_request(self, start=True):
        if start:
            self.get_logger().info("Requesting to START QR Code Detection...")
        else:
            self.get_logger().info("Requesting to STOP QR Code Detection...")

        self.future = self.cli.call_async(self.req)
        return self.future

def main(args=None):
    rclpy.init(args=args)
    client = QrCodeClient()
    future = client.send_request()
    rclpy.spin_until_future_complete(client, future)
    response = future.result()
    client.get_logger().info(
        f'Result: {response.message} ')
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()