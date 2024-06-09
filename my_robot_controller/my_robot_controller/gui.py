import os
import signal
import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import subprocess

class ROS2GUI(Node):
    def __init__(self):
        super().__init__('ros2_gui')
        self.subscri_ = self.create_subscription(String, 'feedback', self.sub_callback, 10)
        self.odom_sub_ = self.create_subscription(Odometry, '/odom', self.odom_sub_callback, 10)
        self.publish_ = self.create_publisher(Int32, 'cmdToControl', 10)
        self.cmd_vel_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.processes = []
        self.param_setup()
        self.gui_setup()

    def param_setup(self):
        self.cond1 = True
        self.cond2 = True
        self.cond3 = False
        self.cond4 = True
        self.cond5 = True
        self.cond6 = True
        self.text_data = [
            "Bring up",
            "Send the work area",
            "Delete the work area",
            "Send the path",
            "Delete the path",
            "Start from the first point",
            "Start from the nearest point",
            "Stop following the path",
            "Continue following the path",
            "Quit"
        ]
        self.process0 = None
        self.process1 = None
        self.process2 = None

    def sub_callback(self, msg : String):
        self.feedback_label.config(text=f"Feedback: {msg.data}")

    def odom_sub_callback(self, msg : Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # Convert quaternion to Euler angles to get theta
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        self.odom_feedback.config(text=f"Position: x={x:.2f}, y={y:.2f}, θ={theta:.2f}")

    def gui_setup(self):
        self.root = tk.Tk()
        self.root.title("Control Panel")
        self.root.protocol("WM_DELETE_WINDOW", self.shutdown_gui)  # Handle window close

        self.root.grid_columnconfigure(0, weight=1, minsize=150)
        self.root.grid_columnconfigure(1, weight=1, minsize=150)
        self.root.grid_columnconfigure(2, weight=1, minsize=150)
        self.root.grid_rowconfigure(0, weight=1, minsize=50)
        self.root.grid_rowconfigure(1, weight=1, minsize=50)
        self.root.grid_rowconfigure(2, weight=1, minsize=50)
        self.root.grid_rowconfigure(3, weight=1, minsize=50)
        self.root.grid_rowconfigure(4, weight=1, minsize=50)
        self.root.grid_rowconfigure(5, weight=1, minsize=50)
        self.root.grid_rowconfigure(6, weight=1, minsize=50)

        # Label 1 trở thành nút bấm
        self.button1 = tk.Button(self.root, text=self.text_data[0], bg="lightblue", width=25, height=2, command=self.button1_callback)
        self.button1.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        # Label 2 trở thành khung hiển thị tin nhắn từ topic đăng ký
        self.feedback_label = tk.Label(self.root, text="Feedback: Waiting for messages...", width=25, height=2, bg="white", relief="groove", bd=2)
        self.feedback_label.grid(row=0, column=1, columnspan=2, padx=10, pady=10, sticky="nsew")

        # Thêm phần điều khiển robot
        self.teleop_frame = tk.Frame(self.root)
        self.teleop_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

        self.teleop_frame.grid(row=1, column=0, columnspan=3, padx=10, pady=10, sticky="nsew")

        self.forward_button = tk.Button(self.teleop_frame, text="↑", command=lambda: self.move_robot("forward"))
        self.forward_button.grid(row=0, column=1)

        self.left_button = tk.Button(self.teleop_frame, text="←", command=lambda: self.move_robot("left"))
        self.left_button.grid(row=1, column=0)

        self.stop_button = tk.Button(self.teleop_frame, text="◼", command=lambda: self.move_robot("stop"))
        self.stop_button.grid(row=1, column=1)

        self.right_button = tk.Button(self.teleop_frame, text="→", command=lambda: self.move_robot("right"))
        self.right_button.grid(row=1, column=2)

        self.backward_button = tk.Button(self.teleop_frame, text="↓", command=lambda: self.move_robot("backward"))
        self.backward_button.grid(row=2, column=1)

        # Thêm label accuracy_feedback để hiển thị độ chính xác của vị trí ước lượng
        self.accuracy_feedback = tk.Label(self.root, text="vel.x:....,vel.z:....", width=25, height=2, bg="white", relief="groove", bd=2)
        self.accuracy_feedback.grid(row=1, column=1, padx=10, pady=10, sticky="nsew")
        self.accuracy_feedback = tk.Label(self.root, text="Accuracy: Calculating...", width=25, height=2, bg="white", relief="groove", bd=2)
        self.accuracy_feedback.grid(row=1, column=2, padx=10, pady=10, sticky="nsew")

        # Label 3 trở thành nút bấm
        self.button2_0 = tk.Button(self.root, text=self.text_data[1], bg="lightblue", width=25, height=2, command=self.button2_0_callback)
        self.button2_0.grid(row=2, column=0, padx=10, pady=10, sticky="nsew")
        self.button2_0.config(state="disabled")

        self.button2_1 = tk.Button(self.root, text=self.text_data[5], bg="lightblue", width=25, height=2, command=self.button2_1_callback)
        self.button2_1.grid(row=2, column=1, padx=10, pady=10, sticky="nsew")
        self.button2_1.config(state="disabled")

        # Thêm label odom_feedback để hiển thị thông tin từ topic odom
        self.odom_feedback = tk.Label(self.root, text="Position: Waiting for odom...", width=25, height=6, bg="white", relief="groove", bd=2)
        self.odom_feedback.grid(row=2, column=2, rowspan=3, padx=10, pady=10, sticky="nsew")

        self.button3_0 = tk.Button(self.root, text=self.text_data[3], bg="lightblue", width=25, height=2, command=self.button3_0_callback)
        self.button3_0.grid(row=3, column=0, padx=10, pady=10, sticky="nsew")
        self.button3_0.config(state="disabled")

        self.button3_1 = tk.Button(self.root, text=self.text_data[6], bg="lightblue", width=25, height=2, command=self.button3_1_callback)
        self.button3_1.grid(row=3, column=1, padx=10, pady=10, sticky="nsew")
        self.button3_1.config(state="disabled")

        self.button4_1 = tk.Button(self.root, text=self.text_data[7], bg="lightblue", width=25, height=2, command=self.button4_1_callback)
        self.button4_1.grid(row=4, column=1, padx=10, pady=10, sticky="nsew")
        self.button4_1.config(state="disabled")

        self.button4_0 = tk.Button(self.root, text=self.text_data[9], bg="lightblue", width=25, height=2, command=self.shutdown_gui)
        self.button4_0.grid(row=4, column=0, padx=10, pady=10, sticky="nsew")

    def button1_callback(self):
        cmd = Int32()
        cmd.data = 1
        self.publish_.publish(cmd)
        print(cmd.data)
        self.button1.config(bg="gray",state="disabled")
        self.button2_0.config(state="normal")
        self.button3_0.config(state="normal")
        self.launch_command_1 = ["ros2", "launch", "my_launch", "my_localization.launch.py"]
        self.launch_command_2 = ["ros2", "run", "my_launch", "work_area_monitor"]
        self.launch_command_3 = ["ros2", "run", "my_launch", "create_path"]
        self.launch_command_4 = ["ros2", "run", "my_launch", "controller_monitor"]
        #self.processes.append(subprocess.Popen(self.launch_command_1))
        self.process0 = subprocess.Popen(self.launch_command_1)
        self.process1 = subprocess.Popen(self.launch_command_3)
        self.process2 = subprocess.Popen(self.launch_command_4)
        
        self.feedback_label.config(text="Feedback: Brought up successfully")
        

    def button2_0_callback(self):
        cmd = Int32()
        if self.button2_0.cget('text') == self.text_data[1]:
            self.button2_0.config(text=self.text_data[2])
            cmd.data = 3
            self.publish_.publish(cmd)
        else:
            self.button2_0.config(text=self.text_data[1])
            cmd.data = 4
            self.publish_.publish(cmd)

        print(cmd.data)

    def button2_1_callback(self):
        cmd = Int32()
        cmd.data = 5
        self.publish_.publish(cmd)
        self.button3_1.config(state="disabled")
        self.button2_1.config(bg="gray",state="disabled")
        print(cmd.data)

    def button3_0_callback(self):
        cmd = Int32()
        if self.button3_0.cget('text') == self.text_data[3]:
            self.button3_0.config(text=self.text_data[4])
            cmd.data = 1
            self.publish_.publish(cmd)
            self.button2_1.config(state="normal")
            self.button3_1.config(state="normal")
            self.button4_1.config(state="normal")
        else:
            self.button3_0.config(text=self.text_data[3])
            self.button2_1.config(bg="lightblue",state="disabled")
            self.button3_1.config(bg="lightblue",state="disabled")
            self.button4_1.config(state="disabled")
            cmd.data = 2
            self.publish_.publish(cmd)

    
        print(cmd.data)

    def button3_1_callback(self):
        cmd = Int32()
        cmd.data = 6
        self.publish_.publish(cmd)
        self.button2_1.config(state="disabled")
        self.button3_1.config(bg="gray",state="disabled")
        print(cmd.data)

    def button4_1_callback(self):
        cmd = Int32()

        if self.button4_1.cget('text') == self.text_data[7]:
            self.button4_1.config(text=self.text_data[8])
            cmd.data = 7
            self.publish_.publish(cmd)
        else:
            self.button4_1.config(text=self.text_data[7])
            cmd.data = 8
            self.publish_.publish(cmd)
    
        print(cmd.data)

    def move_robot(self, direction):
        twist = Twist()
        if direction == "forward":
            twist.linear.x = 0.5
        elif direction == "backward":
            twist.linear.x = -0.5
        elif direction == "left":
            twist.angular.z = 0.5
        elif direction == "right":
            twist.angular.z = -0.5
        elif direction == "stop":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        self.cmd_vel_pub_.publish(twist)
        self.feedback_label.config(text=f"Moving {direction}.")

    def run(self):
        self.root.mainloop()

    def shutdown_gui(self):
        cmd = Int32()
        cmd.data = 9
        self.publish_.publish(cmd)
        print(cmd.data)
        #os.kill(self.process2.pid,15)
        rclpy.shutdown()
        self.root.quit()

def main():
    rclpy.init(args=None)
    gui_publisher = ROS2GUI()
    gui_publisher.run()

if __name__ == '__main__':
    main()