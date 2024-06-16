#ifndef CUSTOM_COSTMAP_PLUGIN_HPP_
#define CUSTOM_COSTMAP_PLUGIN_HPP_

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <vector>

namespace costmap_plugin
{
  class CustomCostmapPlugin : public nav2_costmap_2d::CostmapLayer
  {
  public:
    CustomCostmapPlugin() = default;
    virtual ~CustomCostmapPlugin() = default;

    virtual void onInitialize() override;
    virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y) override;
    virtual void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;
    virtual void reset() override;
    virtual bool isClearable() override;

  private:
    void polygonCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
    void commandCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void updateLineCosts(nav2_costmap_2d::Costmap2D& master_grid, const geometry_msgs::msg::Point& start, const geometry_msgs::msg::Point& end);
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr command_sub_;
    std::vector<geometry_msgs::msg::Point> points_;
    double min_x_;
    double min_y_;
    double max_x_;
    double max_y_;
    bool plugin_initialized_;
  };
}  // namespace costmap_plugin

#endif  // CUSTOM_COSTMAP_PLUGIN_HPP_