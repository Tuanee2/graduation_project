#include "../include/MyCostmapPlugin.hpp"
#include <algorithm>
#include <cmath>

namespace costmap_plugin
{
  void CustomCostmapPlugin::onInitialize()
  {
    current_ = true;
    plugin_initialized_ = true;

    min_x_ = std::numeric_limits<double>::max();
    min_y_ = std::numeric_limits<double>::max(); 
    max_x_ = std::numeric_limits<double>::lowest(); 
    max_y_ = std::numeric_limits<double>::lowest(); 

    if (auto node = node_.lock())
    {
      polygon_sub_ = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
          "work_area", rclcpp::SystemDefaultsQoS(),
          std::bind(&CustomCostmapPlugin::polygonCallback, this, std::placeholders::_1));
      command_sub_ = node->create_subscription<std_msgs::msg::Int32>(
          "/cmdToControl", rclcpp::SystemDefaultsQoS(),
          std::bind(&CustomCostmapPlugin::commandCallback, this, std::placeholders::_1));
    }
    else
    {
      throw std::runtime_error{"Failed to lock node"};
    }

    matchSize();
  }

  void CustomCostmapPlugin::polygonCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
  {
    points_.clear();
    min_x_ = std::numeric_limits<double>::max();
    min_y_ = std::numeric_limits<double>::max();
    max_x_ = std::numeric_limits<double>::lowest();
    max_y_ = std::numeric_limits<double>::lowest();

    for (const auto& p : msg->polygon.points) {
        geometry_msgs::msg::Point point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        points_.push_back(point);
        min_x_ = std::min(min_x_, point.x);
        max_x_ = std::max(max_x_, point.x);
        min_y_ = std::min(min_y_, point.y);
        max_y_ = std::max(max_y_, point.y);
    }
  }

  void CustomCostmapPlugin::commandCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if (msg->data == 4)
    {
      points_.clear();
      min_x_ = std::numeric_limits<double>::max();
      min_y_ = std::numeric_limits<double>::max();
      max_x_ = std::numeric_limits<double>::lowest();
      max_y_ = std::numeric_limits<double>::lowest();

      // Clear the costmap by setting all cells in this layer to free space (low cost)
      auto master_grid = layered_costmap_->getCostmap();
      for (unsigned int i = 0; i < master_grid->getSizeInCellsX(); ++i)
      {
        for (unsigned int j = 0; j < master_grid->getSizeInCellsY(); ++j)
        {
          unsigned char cost = master_grid->getCost(i, j);
          if (cost == nav2_costmap_2d::LETHAL_OBSTACLE)
          {
            master_grid->setCost(i, j, nav2_costmap_2d::FREE_SPACE);
          }
        }
      }
    }
  }

  void CustomCostmapPlugin::updateBounds(double /*origin_x*/, double /*origin_y*/, double /*origin_yaw*/, double* min_x, double* min_y, double* max_x, double* max_y)
  {
    if (!plugin_initialized_ || points_.empty())
    {
      return;
    }

    *min_x = std::min(*min_x, this->min_x_);
    *min_y = std::min(*min_y, this->min_y_);
    *max_x = std::max(*max_x, this->max_x_);
    *max_y = std::max(*max_y, this->max_y_);
  }

  void CustomCostmapPlugin::updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
  {
    if (points_.empty()) return;

    for (size_t i = 0; i < points_.size() - 1; ++i)
    {
      updateLineCosts(master_grid, points_[i], points_[i + 1]);
    }

    // Close the polygon by connecting the last point to the first point
    updateLineCosts(master_grid, points_.back(), points_.front());
  }

  void CustomCostmapPlugin::updateLineCosts(nav2_costmap_2d::Costmap2D& master_grid, const geometry_msgs::msg::Point& start, const geometry_msgs::msg::Point& end)
  {
    int x0, y0, x1, y1;
    master_grid.worldToMapNoBounds(start.x, start.y, x0, y0);
    master_grid.worldToMapNoBounds(end.x, end.y, x1, y1);

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;

    while (true)
    {
      master_grid.setCost(x0, y0, nav2_costmap_2d::LETHAL_OBSTACLE);

      if (x0 == x1 && y0 == y1) break;
      int e2 = err * 2;
      if (e2 > -dy)
      {
        err -= dy;
        x0 += sx;
      }
      if (e2 < dx)
      {
        err += dx;
        y0 += sy;
      }
    }
  }

  void CustomCostmapPlugin::reset()
  {
    if (!plugin_initialized_)
    {
      return;
    }

    points_.clear();
    min_x_ = std::numeric_limits<double>::max();
    min_y_ = std::numeric_limits<double>::max();
    max_x_ = std::numeric_limits<double>::lowest();
    max_y_ = std::numeric_limits<double>::lowest();
  }

  bool CustomCostmapPlugin::isClearable()
  {
    return true;
  }
}  // namespace costmap_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(costmap_plugin::CustomCostmapPlugin, nav2_costmap_2d::Layer)