#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "robot_kf/kalman.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <functional>
using std::placeholders::_1;

class KalmanOdomNode : public rclcpp::Node {
public:
  KalmanOdomNode()
  : Node("kalman_odom_node"),
    kf_(6, 3)
  {
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom_noisy", 10,
      std::bind(&KalmanOdomNode::odomCallback, this, _1));

    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/odom_kf", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    last_time_ = rclcpp::Time(0);

    // Measurement model: x, y, yaw
    kf_.H << 1,0,0,0,0,0,
             0,1,0,0,0,0,
             0,0,1,0,0,0;

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "/odom_kf_path", 10);

    path_.header.frame_id = "odom";

    RCLCPP_INFO(this->get_logger(), "Kalman Odom Node has been started.");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    rclcpp::Time current_time = msg->header.stamp;

    // ===== Init time =====
    if (last_time_.nanoseconds() == 0) {
      last_time_ = current_time;
      RCLCPP_WARN(this->get_logger(), "First odom received, initializing time.");
      return;
    }

    double dt = (current_time - last_time_).seconds();
    if (dt <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "Non-positive dt, skipping update.");
      return;
    }
    last_time_ = current_time;

    // ===== State transition =====
    kf_.F.setIdentity();
    kf_.F(0,3) = dt;
    kf_.F(1,4) = dt;
    kf_.F(2,5) = dt;

    kf_.predict();

    // ===== Extract yaw from odom_noisy =====
    tf2::Quaternion q_in;
    tf2::fromMsg(msg->pose.pose.orientation, q_in);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q_in).getRPY(roll, pitch, yaw);

    // ===== Measurement =====
    Eigen::Vector3d z;
    z << msg->pose.pose.position.x,
         msg->pose.pose.position.y,
         yaw;

    kf_.update(z);

    // ===== Publish Odometry =====
    nav_msgs::msg::Odometry odom_out;
    odom_out.header = msg->header;
    odom_out.child_frame_id = "base_link";

    odom_out.pose.pose.position.x = kf_.x(0);
    odom_out.pose.pose.position.y = kf_.x(1);
    odom_out.pose.pose.position.z = 0.0;

    tf2::Quaternion q_out;
    q_out.setRPY(0.0, 0.0, kf_.x(2));
    q_out.normalize();
    odom_out.pose.pose.orientation = tf2::toMsg(q_out);

    pub_odom_->publish(odom_out);

    // ===== Append to Path =====
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = odom_out.header;
    pose_stamped.pose   = odom_out.pose.pose;

    path_.header.stamp = this->now();
    path_.poses.push_back(pose_stamped);

    path_pub_->publish(path_);

    // ===== Broadcast TF =====
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = odom_out.header.stamp;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_link";

    tf_msg.transform.translation.x = odom_out.pose.pose.position.x;
    tf_msg.transform.translation.y = odom_out.pose.pose.position.y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_out.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf_msg);
  }

  KalmanFilter kf_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Time last_time_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::Path path_;

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KalmanOdomNode>());
  rclcpp::shutdown();
  return 0;
}
