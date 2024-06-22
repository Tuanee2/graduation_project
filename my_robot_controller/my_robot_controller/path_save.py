import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Int32
from nav_msgs.msg import Path
import os
import json

PATH_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'saved_paths.json')

class PathManagerNode(Node):
    def __init__(self):
        super().__init__('path_manager')
        self.path_sub = self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.path_name_sub = self.create_subscription(String, 'path_name', self.path_name_callback, 10)
        self.cmd_sub = self.create_subscription(String, 'cmd', self.cmd_callback, 10)
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        
        self.current_path_name = ""
        self.paths = {}
        self.load_paths_from_file()
        self.is_publishing = False  # Biến điều kiện để kiểm tra trạng thái xuất bản

    def path_callback(self, msg : Path):
        if self.is_publishing:
            return  # Không xử lý bản tin nếu đang xuất bản

        if not self.current_path_name:
            self.get_logger().warn('Received path but no path name set.')
            return
        
        path_data = [{'x': pose, 'y': pose.pose.position.y} for pose in msg.poses]
        self.paths[self.current_path_name] = path_data
        self.save_paths_to_file()
        self.get_logger().info(f'Path saved with name: {self.current_path_name}')
        self.current_path_name = ""

    def path_name_callback(self, msg : String):
        self.current_path_name = msg.data
        self.get_logger().info(f'Received path name: {self.current_path_name}')

    def cmd_callback(self, msg : Int32):
        path_name = msg.data
        if path_name in self.paths:
            self.is_publishing = True
            path_msg = self.convert_path_to_msg(self.paths[path_name])
            self.path_pub.publish(path_msg)
            self.get_logger().info(f'Published path with name: {path_name}')
            self.is_publishing = False  # Đặt lại biến điều kiện sau khi xuất bản
        else:
            self.get_logger().warn(f'Path with name {path_name} not found.')

    def save_paths_to_file(self):
        with open(PATH_FILE, 'w') as f:
            json.dump(self.paths, f)

    def load_paths_from_file(self):
        if os.path.exists(PATH_FILE):
            with open(PATH_FILE, 'r') as f:
                self.paths = json.load(f)

    def convert_path_to_msg(self, path_data):
        path_msg = Path()
        path_msg.header.frame_id = 'map'  # Cập nhật frame_id phù hợp nếu cần
        for point in path_data:
            pose_stamped = Path().poses.create()
            pose_stamped.pose.position.x = point['x']
            pose_stamped.pose.position.y = point['y']
            path_msg.poses.append(pose_stamped)
        return path_msg

def main(args=None):
    rclpy.init(args=args)
    node = PathManagerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()