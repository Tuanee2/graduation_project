import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import tkinter as tk
import math

class TkinterDisplay(Node):
    def __init__(self):
        super().__init__('tkinter_display')

        self.plan_subscriber = self.create_subscription(
            Path,
            'plan',
            self.plan_callback,
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.plan = None
        self.current_pose = None

        self.root = tk.Tk()
        self.root.title("Robot Display")

        self.labels = {
            "next_point": tk.Label(self.root, text="Next Point:"),
            "current_pose": tk.Label(self.root, text="Current Pose:"),
            "distance_error": tk.Label(self.root, text="Distance Error:"),
            "next_angle": tk.Label(self.root, text="Next Angle:"),
            "current_angle": tk.Label(self.root, text="Current Angle:"),
            "angle_error": tk.Label(self.root, text="Angle Error:")
        }

        for label in self.labels.values():
            label.pack()

        self.update_display()
        self.root.mainloop()

    def plan_callback(self, msg):
        self.plan = msg.poses

    def get_current_pose(self):
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.current_pose = PoseStamped()
            self.current_pose.pose.position.x = transform.transform.translation.x
            self.current_pose.pose.position.y = transform.transform.translation.y
            self.current_pose.pose.orientation = transform.transform.rotation
        except Exception as e:
            self.get_logger().info(f"Could not get transform: {e}")

    def calculate_errors(self):
        if self.plan and self.current_pose:
            next_point = self.plan[0].pose  # Giả sử điểm tiếp theo là điểm đầu tiên trong plan

            distance_error = math.sqrt(
                (next_point.position.x - self.current_pose.pose.position.x) ** 2 +
                (next_point.position.y - self.current_pose.pose.position.y) ** 2
            )

            next_angle = math.atan2(
                next_point.position.y - self.current_pose.pose.position.y,
                next_point.position.x - self.current_pose.pose.position.x
            )

            current_orientation = self.current_pose.pose.orientation
            current_angle = self.euler_from_quaternion(
                current_orientation.x,
                current_orientation.y,
                current_orientation.z,
                current_orientation.w
            )

            angle_error = next_angle - current_angle

            return distance_error, next_angle, current_angle, angle_error, next_point
        return None, None, None, None, None

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return yaw_z

    def update_display(self):
        self.get_current_pose()
        distance_error, next_angle, current_angle, angle_error, next_point = self.calculate_errors()

        if next_point:
            self.labels["next_point"].config(text=f"Next Point: ({next_point.position.x:.2f}, {next_point.position.y:.2f})")
        if self.current_pose:
            self.labels["current_pose"].config(text=f"Current Pose: ({self.current_pose.pose.position.x:.2f}, {self.current_pose.pose.position.y:.2f})")
        if distance_error is not None:
            self.labels["distance_error"].config(text=f"Distance Error: {distance_error:.2f}")
        if next_angle is not None:
            self.labels["next_angle"].config(text=f"Next Angle: {math.degrees(next_angle):.2f}°")
        if current_angle is not None:
            self.labels["current_angle"].config(text=f"Current Angle: {math.degrees(current_angle):.2f}°")
        if angle_error is not None:
            self.labels["angle_error"].config(text=f"Angle Error: {math.degrees(angle_error):.2f}°")

        self.root.after(100, self.update_display)

def main(args=None):
    rclpy.init(args=args)
    node = TkinterDisplay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()