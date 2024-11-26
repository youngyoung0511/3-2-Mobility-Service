import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class ArgumentNode(Node):
    def __init__(self):
        super().__init__('argument')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'arithmetic_arguments', 10)
        self.timer = self.create_timer(1.0, self.publish_arguments)

    def publish_arguments(self):
        msg = Float64MultiArray()
        msg.data = [1.0, 2.0, 3.0]  # a, b, c 값
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing arguments: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = ArgumentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
