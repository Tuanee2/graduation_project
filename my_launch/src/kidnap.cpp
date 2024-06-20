#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

using std::placeholders::_1;

class KidnapDetector : public rclcpp::Node
{
public:
  KidnapDetector() : Node("kidnap_detector"), kidnapped_(false), last_kidnap_warn_time_(this->now())
  {
    amcl_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 10, std::bind(&KidnapDetector::pose_callback, this, _1));

    subscription_cmd_ = this->create_subscription<std_msgs::msg::Int32>(
      "/cmdToControl", 10, std::bind(&KidnapDetector::cmd_callback, this, std::placeholders::_1));
    
    accuracy_pub_ = this->create_publisher<std_msgs::msg::Float32>("/acc", 10);
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    double covariance_threshold = 0.01;  // Ngưỡng để xác định vị trí chính xác, điều chỉnh tùy theo thử nghiệm
    double accuracy = calculate_accuracy(msg);
    auto accuracy_msg = std_msgs::msg::Float32();
    accuracy_msg.data = accuracy;
    accuracy_pub_->publish(accuracy_msg);
  }

  double calculate_accuracy(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    double total_accuracy = 0.0;
    double covariances[3] = {msg->pose.covariance[0], msg->pose.covariance[1 * 7], msg->pose.covariance[5 * 7]};
    
    for (double cov : covariances)
    {
      if (cov < 0.01)
      {
        total_accuracy += 100.0;
      }
      else if (cov > 5.0)
      {
        total_accuracy += 0.0;
      }
      else
      {
        total_accuracy += 100*(cov - 5)/(0.01-5);
      }
    }

    return total_accuracy / 3.0;
  }

  void cmd_callback(const std_msgs::msg::Int32::SharedPtr msg){
    if(msg->data == 9){
      rclcpp::shutdown();
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr accuracy_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool kidnapped_;
  rclcpp::Time last_kidnap_warn_time_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KidnapDetector>());
  rclcpp::shutdown();
  return 0;
}