import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
from example_interfaces.srv import SetBool

class CalculatorNode(Node):
    def __init__(self):
        super().__init__('calculator')

        # Subscribers
        self.argument_subscriber = self.create_subscription(
            Float64MultiArray,
            'arithmetic_arguments',
            self.receive_arguments,
            10
        )

        # Service Server
        self.operator_service = self.create_service(
            SetBool,
            'arithmetic_operator',
            self.receive_operator
        )

        # Publisher
        self.result_publisher = self.create_publisher(Float64, 'calculation_result', 10)

        # Internal variables
        self.arguments = []
        self.operator = None

    def receive_arguments(self, msg):
        """Receive arguments from 'arithmetic_arguments' topic."""
        self.arguments = msg.data
        self.get_logger().info(f"Received arguments: {self.arguments}")

    def receive_operator(self, request, response):
        """Receive operator from service and calculate result."""
        self.operator = '+' if request.data else '*'
        self.get_logger().info(f"Received operator: {self.operator}")

        # Calculate if arguments are available
        if self.arguments:
            result = self.calculate()
            self.get_logger().info(f"Calculation result: {result}")

            # Publish the result
            result_msg = Float64()
            result_msg.data = result
            self.result_publisher.publish(result_msg)
            self.get_logger().info("Published calculation result.")
        else:
            self.get_logger().warning("Arguments not received yet.")

        response.success = True
        return response

    def calculate(self):
        """Perform calculation based on operator and arguments."""
        a, b, c = self.arguments
        if self.operator == '+':
            return a + b + c
        elif self.operator == '*':
            return a * b * c
        else:
            self.get_logger().error("Invalid operator.")
            return 0.0

def main(args=None):
    rclpy.init(args=args)
    node = CalculatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Calculator node shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
