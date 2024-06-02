#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <cmath>
#include <mutex>
#include "custom_path/msg/custom_path.hpp"

class PathFollower : public rclcpp::Node
{
public:
    PathFollower() : Node("path_follower"), current_index_(0), following_path_(false), path_received_(false), paused_(false), rotation_done_(false)
    {
        // Subscriber
        subscription_amcl_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10, std::bind(&PathFollower::pose_callback, this, std::placeholders::_1));
        
        subscription_plan_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10, std::bind(&PathFollower::plan_handle, this, std::placeholders::_1));
        
        subscription_cmd_ = this->create_subscription<std_msgs::msg::Int32>(
            "/cmdToControl", 10, std::bind(&PathFollower::cmd_handle, this, std::placeholders::_1));
        
        // Publisher
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        feedback_publisher_ = this->create_publisher<std_msgs::msg::String>("/feedback", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200), std::bind(&PathFollower::timer_callback, this));
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        last_pose_msg_ = msg;
    }

    void timer_callback()
    {
        if (last_pose_msg_ == nullptr) {
            return;
        }
        
        current_pose_ = last_pose_msg_->pose.pose;
        
        //bool inside = isPointInPolygon(path_,current_pose_);
        bool inside = true;
        bool condition = (!following_path_ || paused_ || !inside) ? true : false;
        
        
        if (condition)
        {
            // If not following the path or paused, publish zero velocity
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.0;
            cmd_vel_publisher_->publish(twist_msg);
        }
        else
        {
            std::cout<<"run"<<std::endl;
            // Control to follow the path
            if (current_index_ < path_.size())
            {
                controller(path_[current_index_].pose.position.x, path_[current_index_].pose.position.y);
            }
            else
            {
                following_path_ = false;
                path_received_ = false;
                auto feedback_msg = std_msgs::msg::String();
                feedback_msg.data = "Completed following the path";
                feedback_publisher_->publish(feedback_msg);
            }
        }
    }

    void plan_handle(const nav_msgs::msg::Path::SharedPtr msg)
    {
        
        auto feedback_msg = std_msgs::msg::String();
        mutex_.lock();
        path_ = msg->poses;
        feedback_msg.data = "please watting for process the path data";
        feedback_publisher_->publish(feedback_msg);
        feedback_msg.data = "process complited";
        feedback_publisher_->publish(feedback_msg);
        mutex_.unlock();
        
        current_index_ = 0;  // Reset index
        path_received_ = true;
        following_path_ = false;
        paused_ = false;

        feedback_msg.data = "Received new plan";
        std::cout<<"Received new plan have :"<<path_.size()<<" points"<<std::endl;
        feedback_publisher_->publish(feedback_msg);
    }

    bool isPointInPolygon(std::vector<geometry_msgs::msg::PoseStamped>& polygon, const geometry_msgs::msg::Pose& testPoint) {
        bool result = false;
        int n = polygon.size();
        for (int i = 0, j = n - 1; i < n; j = i++) {
            if (((polygon[i].pose.position.y > testPoint.position.y) != (polygon[j].pose.position.y > testPoint.position.y)) &&
                (testPoint.position.x < (polygon[j].pose.position.x - polygon[i].pose.position.x) * (testPoint.position.y - polygon[i].pose.position.y) / (polygon[j].pose.position.y - polygon[i].pose.position.y) + polygon[i].pose.position.x)) {
                result = !result;
            }
        }
        return result;
    }

    void runAction(){
        
    }

    void cmd_handle(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if (!path_received_ && msg->data != 6)
        {
            // Ignore commands if no path has been received, unless the command is to clear the path
            auto feedback_msg = std_msgs::msg::String();
            feedback_msg.data = "No path received yet";
            feedback_publisher_->publish(feedback_msg);
            return;
        }

        if (msg->data == 2)
        {
            // Start from the first point
            
            current_index_ = 0;
            following_path_ = true;
            paused_ = false;
            std::cout<<"start2"<<std::endl;
            
        }
        else if (msg->data == 3)
        {
            // Start from the nearest point
           
            current_index_ = find_nearest_index();
            following_path_ = true;
            paused_ = false;
            std::cout<<"start3"<<std::endl;
            
        }
        else if (msg->data == 4)
        {
            // Pause the movement
            paused_ = true;
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.0;
            cmd_vel_publisher_->publish(twist_msg);
        }
        else if (msg->data == 5)
        {
            // Resume the movement
            paused_ = false;
        }
        else if (msg->data == 6)
        {
            // Clear the path and stop
           
            path_.clear();
            path_received_ = false;
            following_path_ = false;
            paused_ = false;
           
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.0;
            cmd_vel_publisher_->publish(twist_msg);

            auto feedback_msg = std_msgs::msg::String();
            feedback_msg.data = "Cleared the path";
            feedback_publisher_->publish(feedback_msg);
        }
    }

    int find_nearest_index()
    {
        double min_distance = std::numeric_limits<double>::max();
        int nearest_index = 0;

        for (size_t i = 0; i < path_.size(); ++i)
        {
            double distance = std::sqrt(std::pow(current_pose_.position.x - path_[i].pose.position.x, 2) +
                                        std::pow(current_pose_.position.y - path_[i].pose.position.y, 2));
            if (distance < min_distance)
            {
                min_distance = distance;
                nearest_index = i;
            }
        }
        return nearest_index;
    }

    void controller(double x_goal, double y_goal)
    {
        double Kp_lin = 3;  // Linear velocity proportional gain
        double Kp_ang = 1.0;  // Angular velocity proportional gain
        double threshold = 0.1;  // Distance threshold to consider goal reached
        double yaw_threshold = 0.1;  // Angular error threshold
        //double max_lin_vel = 1.0;  // Maximum linear velocity
        //double max_ang_vel = 1.0;  // Maximum angular velocity

        double dx = x_goal - current_pose_.position.x;
        double dy = y_goal - current_pose_.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        double angle_to_goal = std::atan2(dy, dx);
        double angle_diff = normalize_angle(angle_to_goal - get_yaw(current_pose_.orientation));

        std::cout << "Distance: " << distance << ", Angle Diff: " << angle_diff << std::endl;

        auto twist_msg = geometry_msgs::msg::Twist();

        // Giai đoạn quay để hướng robot về phía mục tiêu
        if (!rotation_done_)
        {
            if (std::abs(angle_diff) > yaw_threshold)
            {
                twist_msg.linear.x = 0.0;
                twist_msg.angular.z = Kp_ang * angle_diff;
                //twist_msg.angular.z = std::min(std::max(twist_msg.angular.z, -max_ang_vel), max_ang_vel);
            }
            else
            {
                rotation_done_ = true;  // Mark rotation as done
            }
        }

        if (rotation_done_)
        {
            if (distance > threshold)
            {
                twist_msg.linear.x = Kp_lin * distance;
                twist_msg.angular.z = Kp_ang * angle_diff;

                // Limit the velocities
                //twist_msg.linear.x = std::min(twist_msg.linear.x, max_lin_vel);
                //twist_msg.angular.z = std::min(std::max(twist_msg.angular.z, -max_ang_vel), max_ang_vel);
            }
            else
            {
                current_index_++;  // Move to the next point
                rotation_done_ = false;  // Reset for the next goal
                return;
            }
        }
        cmd_vel_publisher_->publish(twist_msg);
    }

    double get_yaw(const geometry_msgs::msg::Quaternion &q)
    {
        double roll, pitch, yaw;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        return yaw;
    }

    double normalize_angle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_amcl_pose_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_plan_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_cmd_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<geometry_msgs::msg::PoseStamped> path_;
    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_pose_msg_;
    size_t current_index_;
    bool following_path_;
    bool path_received_;
    bool paused_;
    bool rotation_done_;
    std::mutex mutex_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}