#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <vector>
#include <cmath>
#include <mutex>
#include "custom_path/msg/custom_path.hpp"

class PathFollower : public rclcpp::Node
{
public:
    PathFollower() : Node("path_follower"), current_index_(0), following_path_(false), path_received_(false), paused_(false), rotation_done_(false), loop_(false),tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
    {
        // Subscriber
        //subscription_amcl_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        //    "/amcl_pose", 10, std::bind(&PathFollower::pose_callback, this, std::placeholders::_1));
        
        subscription_plan_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10, std::bind(&PathFollower::plan_handle, this, std::placeholders::_1));
        
        subscription_cmd_ = this->create_subscription<std_msgs::msg::Int32>(
            "/cmdToControl", 10, std::bind(&PathFollower::cmd_handle, this, std::placeholders::_1));
        
        // Publisher
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        feedback_publisher_ = this->create_publisher<std_msgs::msg::String>("/feedback", 10);
        error_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/error", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(25), std::bind(&PathFollower::timer_callback, this));
    }

private:
    void pose_callback()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            // Lấy transform từ "map" đến "base_link"
            transform_stamped = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);

            current_pose_.position.x = transform_stamped.transform.translation.x;
            current_pose_.position.y = transform_stamped.transform.translation.y;
            current_pose_.position.z = transform_stamped.transform.translation.z;
            current_pose_.orientation = transform_stamped.transform.rotation;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }
    }

    void timer_callback()
    {
        // Gọi pose_callback để cập nhật current_pose_
        pose_callback();
        
        // Kiểm tra nếu current_pose_ chưa được cập nhật
        if (current_pose_.position.x == 0.0 && current_pose_.position.y == 0.0) {
            return;
        }
        
        //bool inside = isPointInPolygon(path_,current_pose_);

        bool condition = (!following_path_ || paused_ );
        
        if (condition)
        {
            return;
        }
        else
        {
            if(loop_ == false){
                if (current_index_ < path_.size()-1)
                {
                    
                    controller(path_);
                }else{
                    following_path_ = false;
                    path_received_ = false;
                    auto twist_msg = geometry_msgs::msg::Twist();
                    twist_msg.linear.x = 0;
                    twist_msg.angular.z = 0;
                    cmd_vel_publisher_->publish(twist_msg);
                    auto feedback_msg = std_msgs::msg::String();
                    feedback_msg.data = "Completed following the path";
                    feedback_publisher_->publish(feedback_msg);
                }
            }else{
                if (current_index_ < path_1.size()){
                    controller(path_1);
                }else{
                    current_index_ = 0;
                }
            }
        }
    }

    void plan_handle(const nav_msgs::msg::Path::SharedPtr msg)
    {
        auto feedback_msg = std_msgs::msg::String();
        mutex_.lock();
        path_ = msg->poses;
        mutex_.unlock();
        
        current_index_ = 0;  // Reset index
        path_received_ = true;
        following_path_ = false;
        paused_ = false;

        feedback_msg.data = "Received new plan";
        std::cout << "Received new plan have :" << path_.size() << " points" << std::endl;
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

    void cmd_handle(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if (!path_received_ && msg->data != 9)
        {
            // Bỏ qua các lệnh nếu chưa nhận được đường dẫn, trừ khi lệnh là để xóa đường dẫn
            auto feedback_msg = std_msgs::msg::String();
            feedback_msg.data = "No path received yet";
            feedback_publisher_->publish(feedback_msg);
            return;
        }

        if (msg->data == 5)
        {
            // Bắt đầu từ điểm đầu tiên
            current_index_ = 0;
            following_path_ = true;
            paused_ = false;
            std::cout << "start2" << std::endl;
        }
        else if (msg->data == 6)
        {
            // Bắt đầu từ điểm gần nhất
            current_index_ = find_nearest_index();
            following_path_ = true;
            paused_ = false;
            std::cout << "start3" << std::endl;
        }
        else if (msg->data == 7)
        {
            // Tạm dừng chuyển động
            paused_ = true;
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.0;
            cmd_vel_publisher_->publish(twist_msg);
        }
        else if (msg->data == 8)
        {
            // Tiếp tục chuyển động
            paused_ = false;
        }
        else if (msg->data == 2)
        {
            // Xóa đường dẫn và dừng lại
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
        else if (msg->data == 10)
        {
            loop_ = true;
            for(int j=0;j<15;j++){
                for(int i=0;i<path_.size();i++){
                    path_1.push_back(path_[i]);
                }
            }
        }
        else if (msg->data == 11)
        {
            loop_ = false;
        }
        else if (msg->data == 9){
            rclcpp::shutdown();
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

    void controller(std::vector<geometry_msgs::msg::PoseStamped> path_)
    {
        double delta_x = path_[current_index_+1].pose.position.x - path_[current_index_].pose.position.x;
        double delta_y = path_[current_index_+1].pose.position.y - path_[current_index_].pose.position.y;
        double d_x = current_pose_.position.x - path_[current_index_].pose.position.x;
        double d_y = current_pose_.position.y - path_[current_index_].pose.position.y;
        // H = [delta_x;delta_y] Hn = [-delta_y ; delta_x]; r = [d_x ; d_y];
        double e_p = (-delta_y*d_x+delta_x*d_y)/(delta_x*delta_x+delta_y*delta_y); // e_p = (Hn'.r)/(Hn'.Hn)
        double u = (delta_x*d_x+delta_y*d_y)/(delta_x*delta_x+delta_y*delta_y);// u =(H'.r)/(H'.H)
        auto error_msg = std_msgs::msg::Float32();
        error_msg.data = (float)e_p;
        error_publisher_->publish(error_msg);
        if (u > 1)
        {
            current_index_ ++;
            double delta_x = path_[current_index_+1].pose.position.x - path_[current_index_].pose.position.x;
            double delta_y = path_[current_index_+1].pose.position.y - path_[current_index_].pose.position.y;
            double d_x = current_pose_.position.x - path_[current_index_].pose.position.x;
            double d_y = current_pose_.position.y - path_[current_index_].pose.position.y;
            
            double e_p = (-delta_y*d_x+delta_x*d_y)/(delta_x*delta_x+delta_y*delta_y);
            double u = (delta_x*d_x+delta_y*d_y)/(delta_x*delta_x+delta_y*delta_y); 
        }
        double theta_path = atan2(delta_y,delta_x);
        double theta_ed = -atan2(5*e_p,1);
        double angle_diff = normalize_angle(normalize_angle(theta_path+theta_ed) - get_yaw(current_pose_.orientation));
        // Controller
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = 0.1*cos(angle_diff);
        twist_msg.angular.z = 0.75 * angle_diff;
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
        while(angle > M_PI || angle < -M_PI){
            angle = (angle > M_PI) ? (angle - 2*M_PI) : ((angle < -M_PI) ? (angle + 2*M_PI) : angle);
        }
        return angle;
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_amcl_pose_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_plan_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_cmd_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr error_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<geometry_msgs::msg::PoseStamped> path_;
    std::vector<geometry_msgs::msg::PoseStamped> path_1;
    geometry_msgs::msg::Pose current_pose_;
    //geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_pose_msg_;
    size_t current_index_;
    bool following_path_;
    bool path_received_;
    bool paused_;
    bool rotation_done_;
    bool loop_;
    std::mutex mutex_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
