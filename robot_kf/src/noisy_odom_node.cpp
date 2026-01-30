#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <random>

using std::placeholders::_1;

class NoisyOdomNode : public rclcpp::Node
{
public:
    NoisyOdomNode() 
    : Node("noisy_odom_node"),
      gen_(rd_()),
      noise_pos_(0.0, 0.05),  
      noise_theta_(0.0, 0.02)
    {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&NoisyOdomNode::odomCallback, this, _1));

    noisy_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/odom_noisy", 10);

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "/odom_noisy_path", 10);

    path_.header.frame_id = "odom";

    RCLCPP_INFO(this->get_logger(), "Noisy Odom Node has been started.");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
    auto noisy_msg = *msg;

    noisy_msg.pose.pose.position.x += noise_pos_(gen_);
    noisy_msg.pose.pose.position.y += noise_pos_(gen_);

    tf2::Quaternion q_orig;
    tf2::fromMsg(msg->pose.pose.orientation, q_orig);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q_orig).getRPY(roll, pitch, yaw);

    double noisy_yaw = yaw + noise_theta_(gen_);

    tf2::Quaternion q_noisy;
    q_noisy.setRPY(0.0, 0.0, noisy_yaw);
    q_noisy.normalize();

    noisy_msg.pose.pose.orientation = tf2::toMsg(q_noisy);

    noisy_odom_pub_->publish(noisy_msg);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = noisy_msg.header;
    pose.pose   = noisy_msg.pose.pose;

    path_.header.stamp = this->now();
    path_.poses.push_back(pose);

    path_pub_->publish(path_);
    }


    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr noisy_odom_pub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_;

    std::random_device rd_;
    std::mt19937 gen_;
    std::normal_distribution<double> noise_pos_;
    std::normal_distribution<double> noise_theta_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NoisyOdomNode>());
    rclcpp::shutdown();
    return 0;
}