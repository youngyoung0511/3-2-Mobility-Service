import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64  # 결과를 Float64로 퍼블리시
from std_msgs.msg import String  # 바운더리 검사 결과를 출력

class CheckerNode(Node):
    def __init__(self):
        super().__init__('checker')

        # 구독: Calculator로부터 계산 결과를 받음
        self.subscription = self.create_subscription(
            Float64,
            'calculation_result',
            self.check_result,
            10
        )

        # 퍼블리시: 검사 결과를 퍼블리시
        self.result_publisher = self.create_publisher(String, 'check_result', 10)

        # 바운더리 설정
        self.upper_limit = 100.0
        self.lower_limit = 10.0
        self.get_logger().info(f"Checker node initialized with Lower={self.lower_limit}, Upper={self.upper_limit}")

    def check_result(self, msg):
        """Calculator에서 보낸 결과를 검사하는 함수"""
        result = msg.data
        self.get_logger().info(f"Received result: {result}")

        # 바운더리 검사
        if self.lower_limit <= result <= self.upper_limit:
            feedback = f"Result {result} is within bounds ({self.lower_limit}, {self.upper_limit})."
            self.get_logger().info(feedback)
        else:
            feedback = f"Result {result} is out of bounds ({self.lower_limit}, {self.upper_limit})."
            self.get_logger().warning(feedback)

        # 검사 결과를 퍼블리시
        feedback_msg = String()
        feedback_msg.data = feedback
        self.result_publisher.publish(feedback_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CheckerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
