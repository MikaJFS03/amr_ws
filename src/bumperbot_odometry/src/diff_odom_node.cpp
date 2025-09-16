// diff_odom_node.cpp
#include <cmath>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;

class DiffOdomNode : public rclcpp::Node {
public:
  DiffOdomNode()
  : Node("diff_odom_node"),
    left_ticks_(0), right_ticks_(0),
    last_left_ticks_(0), last_right_ticks_(0),
    x_(0.0), y_(0.0), theta_(0.0)
  {
    // Parameters (tune these)
    this->declare_parameter<double>("wheel_radius", 0.075); // meters
    this->declare_parameter<double>("wheel_base",   0.370); // meters (distance between wheels)
    this->declare_parameter<int>("ticks_per_rev", 400);
    this->declare_parameter<double>("publish_hz", 50.0);
    this->declare_parameter<std::string>("left_topic", "/left_encoder");
    this->declare_parameter<std::string>("right_topic", "/right_encoder");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_link");

    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    wheel_base_   = this->get_parameter("wheel_base").as_double();
    ticks_per_rev_= this->get_parameter("ticks_per_rev").as_int();
    double hz      = this->get_parameter("publish_hz").as_double();
    left_topic_    = this->get_parameter("left_topic").as_string();
    right_topic_   = this->get_parameter("right_topic").as_string();
    odom_frame_    = this->get_parameter("odom_frame").as_string();
    base_frame_    = this->get_parameter("base_frame").as_string();

    meters_per_tick_ = (2.0 * M_PI * wheel_radius_) / static_cast<double>(ticks_per_rev_);

    // subs
    left_sub_  = this->create_subscription<std_msgs::msg::Int64>(
      left_topic_, 10, std::bind(&DiffOdomNode::leftCallback, this, _1));
    right_sub_ = this->create_subscription<std_msgs::msg::Int64>(
      right_topic_, 10, std::bind(&DiffOdomNode::rightCallback, this, _1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    last_time_ = this->now();
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0/hz),
      std::bind(&DiffOdomNode::timerCallback, this));
  }

private:
  void leftCallback(const std_msgs::msg::Int64::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    left_ticks_ = msg->data;
  }

  void rightCallback(const std_msgs::msg::Int64::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    right_ticks_ = msg->data;
  }

  void timerCallback() {
    rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    if (dt <= 0.0) return;

    int64_t left, right;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      left  = left_ticks_;
      right = right_ticks_;
    }

    int64_t dl_ticks = left - last_left_ticks_;
    int64_t dr_ticks = right - last_right_ticks_;

    double dl = static_cast<double>(dl_ticks) * meters_per_tick_;
    double dr = static_cast<double>(dr_ticks) * meters_per_tick_;

    // differential drive kinematics
    double d_center = (dl + dr) / 2.0;
    double d_theta  = (dr - dl) / wheel_base_;

    // integrate pose (use midpoint approximation)
    double mid_theta = theta_ + d_theta / 2.0;
    x_ += d_center * std::cos(mid_theta);
    y_ += d_center * std::sin(mid_theta);
    theta_ += d_theta;

    // normalize theta between -pi..pi
    theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

    double vx = d_center / dt;
    double vth = d_theta / dt;

    // Publish odometry message
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    odom.pose.pose.orientation = tf2::toMsg(q);

    // example covariances (tune according to your sensors)
    // order: [x y z roll pitch yaw] -> 6x6 row-major
    for (int i = 0; i < 36; ++i) odom.pose.covariance[i] = 0.0;
    odom.pose.covariance[0] = 0.02;  // var(x)
    odom.pose.covariance[7] = 0.02;  // var(y)
    odom.pose.covariance[35] = 0.05; // var(yaw)

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = vth;

    // example twist covariance
    for (int i = 0; i < 36; ++i) odom.twist.covariance[i] = 0.0;
    odom.twist.covariance[0] = 0.1;   // var(vx)
    odom.twist.covariance[35] = 0.1;  // var(omega)

    odom_pub_->publish(odom);

    // broadcast tf odom -> base_link
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = odom_frame_;
    t.child_frame_id = base_frame_;
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;
    t.transform.rotation = odom.pose.pose.orientation;

//  tf_broadcaster_->sendTransform(t);
//  proposed change: also send base_footprint -> base_link with static offset in z so the urdf is correct/usable

    geometry_msgs::msg::TransformStamped t_base_link;
    t_base_link.header.stamp = t.header.stamp;
    t_base_link.header.frame_id = "base_footprint";   // parent
    t_base_link.child_frame_id = "base_link";         // child
    t_base_link.transform.translation.x = 0.0;
    t_base_link.transform.translation.y = 0.0;
    t_base_link.transform.translation.z = 0.033;      // dari URDF: origin z=0.033
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    t_base_link.transform.rotation = tf2::toMsg(q);

    // send both (tf2_ros::TransformBroadcaster has overload that takes vector)
    tf_broadcaster_->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{t, t_base_link});

    // save for next iter
    last_left_ticks_ = left;
    last_right_ticks_ = right;
    last_time_ = now;
  }

  // params
  double wheel_radius_, wheel_base_, meters_per_tick_;
  int ticks_per_rev_;
  std::string left_topic_, right_topic_, odom_frame_, base_frame_;

  // state
  std::mutex mutex_;
  int64_t left_ticks_, right_ticks_;
  int64_t last_left_ticks_, last_right_ticks_;
  double x_, y_, theta_;
  rclcpp::Time last_time_;

  // ROS elements
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr left_sub_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr right_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiffOdomNode>());
  rclcpp::shutdown();
  return 0;
}
