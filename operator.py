import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class OperatorNode(Node):
    def __init__(self):
        super().__init__('operator')
        self.client = self.create_client(SetBool, 'arithmetic_operator')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.send_request()

    def send_request(self):
        req1 = SetBool.Request()
        req2 = SetBool.Request()
        req1.data = True  # 예: 덧셈
        req2.data = False  # 예: 곱셈
        self.client.call_async(req1)
        self.client.call_async(req2)
        self.get_logger().info('Sent operators: + and *')

def main(args=None):
    rclpy.init(args=args)
    node = OperatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
