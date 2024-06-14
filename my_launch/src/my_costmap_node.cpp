#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_util/lifecycle_node.hpp>

class MyCostmapNode : public nav2_util::LifecycleNode
{
public:
    MyCostmapNode()
    : nav2_util::LifecycleNode("my_costmap_node")
    {
        RCLCPP_INFO(get_logger(), "Initializing MyCostmapNode...");
    }

    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
    {
        RCLCPP_INFO(get_logger(), "Configuring MyCostmapNode...");

        global_costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
        global_costmap_->on_configure(state);

        local_costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap");
        local_costmap_->on_configure(state);

        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
    {
        RCLCPP_INFO(get_logger(), "Activating MyCostmapNode...");

        global_costmap_->on_activate(state);
        local_costmap_->on_activate(state);

        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
    {
        RCLCPP_INFO(get_logger(), "Deactivating MyCostmapNode...");

        global_costmap_->on_deactivate(state);
        local_costmap_->on_deactivate(state);

        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override
    {
        RCLCPP_INFO(get_logger(), "Cleaning up MyCostmapNode...");

        global_costmap_->on_cleanup(state);
        local_costmap_->on_cleanup(state);

        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override
    {
        RCLCPP_INFO(get_logger(), "Shutting down MyCostmapNode...");

        global_costmap_->on_shutdown(state);
        local_costmap_->on_shutdown(state);

        return nav2_util::CallbackReturn::SUCCESS;
    }

private:
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> global_costmap_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> local_costmap_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyCostmapNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}