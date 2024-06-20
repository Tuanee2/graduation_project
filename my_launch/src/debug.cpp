#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class PathFollower : public rclcpp::Node
{
public:
    PathFollower()
        : Node("path_follower_01"),
          tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
          tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
    {
        plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10, std::bind(&PathFollower::planCallback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            500ms, std::bind(&PathFollower::timerCallback, this));
    }

private:
    void planCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        path_ = msg;
    }

    void timerCallback()
    {
        if (!path_ || path_->poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No path received or path is empty");
            return;
        }

        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_->lookupTransform("base_link", "map", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform base_link to map: %s", ex.what());
            return;
        }

        double robot_x = transform_stamped.transform.translation.x;
        double robot_y = transform_stamped.transform.translation.y;
        double robot_yaw = tf2::getYaw(transform_stamped.transform.rotation);

        auto target_pose = path_->poses.front().pose;
        double target_x = target_pose.position.x;
        double target_y = target_pose.position.y;

        double dx = target_x - robot_x;
        double dy = target_y - robot_y;
        double distance_error = std::sqrt(dx * dx + dy * dy);
        double angle_to_target = std::atan2(dy, dx);
        double angle_error = angle_to_target - robot_yaw;

        RCLCPP_INFO(this->get_logger(), "Distance error: %.2f", distance_error);

        if (distance_error < 0.1) // Assuming 0.1 meters as the threshold to consider reached
        {
            RCLCPP_INFO(this->get_logger(), "Angle error: %.2f", angle_error);
            path_->poses.erase(path_->poses.begin()); // Remove the reached point
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
    nav_msgs::msg::Path::SharedPtr path_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollower>());
    rclcpp::shutdown();
    return 0;
}