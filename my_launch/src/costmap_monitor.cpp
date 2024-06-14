#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  // Sử dụng header mới
#include "std_msgs/msg/int32.hpp"

class CostmapMonitor : public rclcpp::Node
{
public:
    CostmapMonitor()
    : Node("costmap_monitor"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/local_costmap/costmap", 10, std::bind(&CostmapMonitor::costmapCallback, this, std::placeholders::_1));
        cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>("/cmdToControl", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&CostmapMonitor::checkCostmap, this));
    }

private:
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        costmap_ = msg;
    }

    void checkCostmap()
    {
        if (!costmap_) {
            return;
        }

        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform base_link to map: %s", ex.what());
            return;
        }

        // Lấy vị trí hiện tại của robot từ transform
        double x = transform.transform.translation.x;
        double y = transform.transform.translation.y;

        // Chuyển đổi vị trí sang tọa độ của costmap
        double resolution = costmap_->info.resolution;
        double origin_x = costmap_->info.origin.position.x;
        double origin_y = costmap_->info.origin.position.y;
        int grid_x = static_cast<int>((x - origin_x) / resolution);
        int grid_y = static_cast<int>((y - origin_y) / resolution);

        // Kiểm tra giá trị chi phí tại vị trí của robot
        if (grid_x >= 0 && grid_x < static_cast<int>(costmap_->info.width) && grid_y >= 0 && grid_y < static_cast<int>(costmap_->info.height)) {
            int index = grid_y * costmap_->info.width + grid_x;
            int cost = costmap_->data[index];

            RCLCPP_INFO(this->get_logger(), "Cost at robot position: %d", cost);

            if (cost > 80) {  // Ngưỡng chi phí cao
                RCLCPP_WARN(this->get_logger(), "High cost area detected! Publishing stop command.");
                publishStopCommand();
            }
        }
    }

    void publishStopCommand()
    {
        auto msg = std_msgs::msg::Int32();
        msg.data = 7;
        cmd_pub_->publish(msg);
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapMonitor>());
    rclcpp::shutdown();
    return 0;
}