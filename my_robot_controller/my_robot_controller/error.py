import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import tf2_ros
from geometry_msgs.msg import TransformStamped

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
ERROR_FILE = os.path.join(CURRENT_DIR, "error.txt")
X_POS_FILE = os.path.join(CURRENT_DIR, "x_pos.txt")
Y_POS_FILE = os.path.join(CURRENT_DIR, "y_pos.txt")

class ErrorListener(Node):
    def __init__(self):
        super().__init__('error_listener')
        self.subscription = self.create_subscription(
            Float32,
            '/error',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Kiểm tra và tạo file nếu chưa tồn tại
        self.get_logger().info(f"Checking if {ERROR_FILE}, {X_POS_FILE}, and {Y_POS_FILE} exist...")
        if not os.path.exists(ERROR_FILE):
            self.get_logger().info(f"{ERROR_FILE} does not exist. Creating...")
            with open(ERROR_FILE, 'w') as file:
                pass
        
        if not os.path.exists(X_POS_FILE):
            self.get_logger().info(f"{X_POS_FILE} does not exist. Creating...")
            with open(X_POS_FILE, 'w') as file:
                pass
        
        if not os.path.exists(Y_POS_FILE):
            self.get_logger().info(f"{Y_POS_FILE} does not exist. Creating...")
            with open(Y_POS_FILE, 'w') as file:
                pass
        
        self.get_logger().info(f"Opening {ERROR_FILE}, {X_POS_FILE}, and {Y_POS_FILE} for appending...")
        self.error_file = open(ERROR_FILE, 'a')  # Mở file ở chế độ append
        self.x_pos_file = open(X_POS_FILE, 'a')  # Mở file ở chế độ append
        self.y_pos_file = open(Y_POS_FILE, 'a')  # Mở file ở chế độ append

    def listener_callback(self, msg: Float32):
        self.get_logger().info('Received error: %f' % msg.data)
        self.error_file.write('%f\n' % msg.data)
        self.record_real_position()

    def record_real_position(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pos = transform.transform.translation
            self.get_logger().info(f"Real position - x: {pos.x}, y: {pos.y}")
            self.x_pos_file.write(f'{pos.x}\n')
            self.y_pos_file.write(f'{pos.y}\n')
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Could not transform base_link to map: {e}")

    def destroy_node(self):
        self.get_logger().info(f"Closing {ERROR_FILE}, {X_POS_FILE}, and {Y_POS_FILE}...")
        self.error_file.close()
        self.x_pos_file.close()
        self.y_pos_file.close()
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