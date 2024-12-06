#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using std::placeholders::_1;

class FriendPose : public rclcpp::Node
{
  public:
    FriendPose()
    : Node("friend_pose")
    {
        friend_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/robot_name/pose", 10, std::bind(&FriendPose::friend_pose_callback, this, _1));
        friend_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/friend_pointcloud", 10);
    }

  private:
    void friend_pose_callback(const geometry_msgs::msg::PoseStamped & msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Get pose: %f, %f, %f, publishing to map ...", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);

        auto msg_pub = sensor_msgs::msg::PointCloud2();

        // make fields for pointcloud2
        std::vector<sensor_msgs::msg::PointField> fields(3);
        fields[0].name = "x";
        fields[0].offset = 0;
        fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        fields[0].count = 1;
        fields[1].name = "y";
        fields[1].offset = 4;
        fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        fields[1].count = 1;
        fields[2].name = "z";
        fields[2].offset = 8;
        fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        fields[2].count = 1;

        // make a cylinder shape
        float radius = 0.42f;
        float height = 1.0f;
        int num_points_circle = 100;
        int num_levels = 10;
        std::vector<float> points;
        for (int i = 0; i <= num_levels; ++i)
        {
            float z = i * (height / num_levels) - height / 2.0f;
            for (int j = 0; j < num_points_circle; ++j)
            {
                float angle = 2.0f * M_PI * j / num_points_circle;
                float x = radius * std::cos(angle);
                float y = radius * std::sin(angle);
                points.push_back(x + msg.pose.position.x);
                points.push_back(y + msg.pose.position.y);
                points.push_back(z + msg.pose.position.z);
            }
        }
        std::vector<uint8_t> cloud_data(points.size() * sizeof(float));
        std::memcpy(cloud_data.data(), points.data(), cloud_data.size());
        
        // publishing to pointcloud2 msg
        msg_pub.header.frame_id = "map";
        msg_pub.header.stamp = this->now();
        msg_pub.fields = fields;
        msg_pub.data = cloud_data;
        msg_pub.height = 1;
        msg_pub.width = points.size() / 3;
        msg_pub.is_dense = true;
        msg_pub.is_bigendian = false;
        msg_pub.point_step = 12;
        msg_pub.row_step = msg_pub.point_step * msg_pub.width;
        friend_pointcloud_publisher_->publish(msg_pub);
    }
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr friend_pose_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr friend_pointcloud_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FriendPose>());
    rclcpp::shutdown();
    return 0;
}