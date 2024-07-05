import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PARENT_DIR = os.path.dirname(CURRENT_DIR)
ERROR_FILE = os.path.join(PARENT_DIR, "error.txt")

class ErrorListener(Node):
    def __init__(self):
        super().__init__('error_listener')
        self.subscription = self.create_subscription(
            Float32,
            '/error',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning
        
        # Kiểm tra và tạo file nếu chưa tồn tại
        if not os.path.exists(ERROR_FILE):
            with open(ERROR_FILE, 'w') as file:
                pass
        
        self.file = open(ERROR_FILE, 'a')  # Mở file ở chế độ append

    def listener_callback(self, msg: Float32):
        self.get_logger().info('Received error: %f' % msg.data)
        self.file.write('%f\n' % msg.data)

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ErrorListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()