#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

class NavigationNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

    NavigationNode()
    : Node("navigation_node")
    {
        navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        follow_waypoints_client_ = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");

        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("send_goal_pose", 10, std::bind(&NavigationNode::goal_callback, this, std::placeholders::_1));
        action_status_pub_ = this->create_publisher<std_msgs::msg::Bool>("/navigation_action_status", 10);

    }

    void goal_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // Determine the action based on input data size
        if (msg->data.size() == 3) {
            // Navigate to a single goal
            RCLCPP_INFO(this->get_logger(), "Try sending goal to action ...");
            send_navigate_to_pose_goal(msg->data[0], msg->data[1], msg->data[2]);
        } else if (msg->data.size() % 3 == 0) {
            // Follow multiple waypoints
            RCLCPP_INFO(this->get_logger(), "Try sending waypoints to action ...");
            std::vector<double> waypoints(msg->data.begin(), msg->data.end());
            send_follow_waypoints_goal(waypoints);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid goal data received");
        }
    }

    void send_navigate_to_pose_goal(double x, double y, double w)
    {
        if (!navigate_to_pose_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.w = w;

        std::stringstream message_stream;
        message_stream << "x: " << float(goal_msg.pose.pose.position.x ) << ", y: " << float(goal_msg.pose.pose.position.y) << ", w: " << float(goal_msg.pose.pose.orientation.w) << ",\n";
        RCLCPP_INFO_STREAM(rclcpp::get_logger("RobotInterface"), "Sending goal: " << message_stream.str());

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&NavigationNode::navigate_to_pose_response_callback, this, std::placeholders::_1);
        send_goal_options.result_callback = std::bind(&NavigationNode::navigate_to_pose_result_callback, this, std::placeholders::_1);

        navigate_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void send_follow_waypoints_goal(const std::vector<double>& waypoints)
    {
        if (!follow_waypoints_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Follow Waypoints action server not available");
            return;
        }

        auto goal_msg = FollowWaypoints::Goal();
        for (size_t i = 0; i < waypoints.size(); i += 3) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = waypoints[i];
            pose.pose.position.y = waypoints[i + 1];
            pose.pose.orientation.w = waypoints[i + 2];
            goal_msg.poses.push_back(pose);
        }

        RCLCPP_INFO(this->get_logger(), "Sending waypoints to action ...");

        auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&NavigationNode::follow_waypoints_response_callback, this, std::placeholders::_1);
        send_goal_options.result_callback = std::bind(&NavigationNode::follow_waypoints_result_callback, this, std::placeholders::_1);

        RCLCPP_INFO(this->get_logger(), "Sending Follow Waypoints goal");
        follow_waypoints_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    void navigate_to_pose_response_callback(std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void follow_waypoints_response_callback(std::shared_ptr<GoalHandleFollowWaypoints> goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void navigate_to_pose_result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        std_msgs::msg::Bool status_msg;
        
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
            status_msg.data = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            status_msg.data = false;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            status_msg.data = false;
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            status_msg.data = false;
            break;
        }

        action_status_pub_->publish(status_msg);
    }

    void follow_waypoints_result_callback(const GoalHandleFollowWaypoints::WrappedResult & result)
    {
        std_msgs::msg::Bool status_msg;

        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
            status_msg.data = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            status_msg.data = false;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            status_msg.data = false;
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            status_msg.data = false;
            break;
        }

        action_status_pub_->publish(status_msg);
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
    rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_waypoints_client_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr action_status_pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
