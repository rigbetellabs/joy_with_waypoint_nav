#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "rclcpp/time_source.hpp"
#include "action_msgs/srv/cancel_goal.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std;

enum class GoalStatus
{
    HOME,
    X,
    Y,
    NONE
};

class AutoJoyTeleop : public rclcpp::Node
{
public:
    AutoJoyTeleop() : Node("auto_joy_teleop")
    {

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&AutoJoyTeleop::joy_callback, this, placeholders::_1));
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
        cancel_goal_client_ = this->create_client<action_msgs::srv::CancelGoal>("/navigate_to_pose/_action/cancel_goal");
        timer_ = this->create_wall_timer(20ms, std::bind(&AutoJoyTeleop::master_callback, this));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        a_scale_ = 0.5;
        l_scale_ = 0.5;

        increment_ = 0.01;

        goal_status_ = GoalStatus::NONE;

        log_interval_ = 2000;

        home_.header.frame_id = "map";
        home_.header.stamp = this->now();
        home_.pose.position.x = 0.0;
        home_.pose.position.y = 0.5;
        home_.pose.position.z = 0.0;
        home_.pose.orientation.x = 0.0;
        home_.pose.orientation.y = 0.0;
        home_.pose.orientation.z = 0.0;
        home_.pose.orientation.w = 1.0;

        RCLCPP_INFO(this->get_logger(), "[NODE INITIATED]");
    }

private:
    void master_callback()
    {
        // log_info();

        auto robot_vel = geometry_msgs::msg::Twist();
        robot_vel.linear.x = x_vel_;
        robot_vel.linear.y = y_vel_;
        robot_vel.linear.z = 0.0;
        robot_vel.angular.x = 0.0;
        robot_vel.angular.y = 0.0;
        robot_vel.angular.z = z_vel_;

        cmd_vel_pub_->publish(robot_vel);
    }

    void joy_callback(const sensor_msgs::msg::Joy &joy_msg)
    {
        int x_axis = 1;
        int y_axis = 0;
        int z_axis = 3;
        int l_inc = 7;
        int a_inc = 6;
        l_scale_ += joy_msg.axes[l_inc] * increment_;
        a_scale_ += joy_msg.axes[a_inc] * increment_;
        x_vel_ = joy_msg.axes[2] < 0.0 ? l_scale_ * joy_msg.axes[x_axis] : 0.0;
        y_vel_ = joy_msg.axes[2] < 0.0 ? l_scale_ * joy_msg.axes[y_axis] : 0.0;
        z_vel_ = joy_msg.axes[2] < 0.0 ? a_scale_ * joy_msg.axes[z_axis] : 0.0;

        if (joy_msg.buttons[0] && goal_status_ != GoalStatus::HOME)
        {
            goal_pub_->publish(home_);
            goal_status_ = GoalStatus::HOME;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Setting Robot Goal: HOME");
        }
        else if (joy_msg.buttons[1] && goal_status_ != GoalStatus::NONE)
        {
            auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();

            auto result = cancel_goal_client_->async_send_request(request);

            goal_status_ = GoalStatus::NONE;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Cancelling Current Goal");
        }
        else if (joy_msg.buttons[2] && goal_status_ != GoalStatus::X)
        {
            goal_pub_->publish(x_goal_);
            goal_status_ = GoalStatus::X;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Setting Robot Goal: X");
        }
        else if (joy_msg.buttons[3] && goal_status_ != GoalStatus::Y)
        {
            goal_pub_->publish(y_goal_);
            goal_status_ = GoalStatus::Y;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Setting Robot Goal: Y");
        }

        if (joy_msg.buttons[4])
        {
            geometry_msgs::msg::TransformStamped t;
            try
            {
                t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

                x_goal_.header.frame_id = "map";
                x_goal_.header.stamp = this->now();
                x_goal_.pose.position.x = t.transform.translation.x;
                x_goal_.pose.position.y = t.transform.translation.y;
                x_goal_.pose.position.z = 0.0;
                x_goal_.pose.orientation.x = 0.0;
                x_goal_.pose.orientation.y = 0.0;
                x_goal_.pose.orientation.z = t.transform.rotation.z;
                x_goal_.pose.orientation.w = t.transform.rotation.w;

                double roll, pitch, yaw;

                tf2::Quaternion quats(x_goal_.pose.orientation.x, x_goal_.pose.orientation.y, x_goal_.pose.orientation.z, x_goal_.pose.orientation.w);
                tf2::Matrix3x3(quats).getRPY(roll, pitch, yaw);

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Stored X pose: (%f, %f, %f)", x_goal_.pose.position.x, x_goal_.pose.position.y, yaw);
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Could not transform %s to %s: %s", "base_link", "map", ex.what());
            }
        }
        else if (joy_msg.buttons[5])
        {
            geometry_msgs::msg::TransformStamped t;
            try
            {
                t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

                y_goal_.header.frame_id = "map";
                y_goal_.header.stamp = this->now();
                y_goal_.pose.position.x = t.transform.translation.x;
                y_goal_.pose.position.y = t.transform.translation.y;
                y_goal_.pose.position.z = 0.0;
                y_goal_.pose.orientation.x = 0.0;
                y_goal_.pose.orientation.y = 0.0;
                y_goal_.pose.orientation.z = t.transform.rotation.z;
                y_goal_.pose.orientation.w = t.transform.rotation.w;

                double roll, pitch, yaw;

                tf2::Quaternion quats(y_goal_.pose.orientation.x, y_goal_.pose.orientation.y, y_goal_.pose.orientation.z, y_goal_.pose.orientation.w);
                tf2::Matrix3x3(quats).getRPY(roll, pitch, yaw);

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Stored Y pose: (%f, %f, %f)", y_goal_.pose.position.x, y_goal_.pose.position.y, yaw);
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Could not transform %s to %s: %s", "base_link", "map", ex.what());
            }
        }
    }

    void log_info()
    {

        if (static_cast<int>(goal_status_) == 0)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Current Goal: HOME");
        }
        else if (static_cast<int>(goal_status_) == 1)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Current Goal: X");
        }
        else if (static_cast<int>(goal_status_) == 2)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Current Goal: Y");
        }
        else if (static_cast<int>(goal_status_) == 3)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Current Goal: NONE");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr cancel_goal_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    float a_scale_;
    float l_scale_;
    float x_vel_;
    float y_vel_;
    float z_vel_;
    float increment_;

    int log_interval_;

    GoalStatus goal_status_;

    geometry_msgs::msg::PoseStamped home_;
    geometry_msgs::msg::PoseStamped x_goal_;
    geometry_msgs::msg::PoseStamped y_goal_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoJoyTeleop>());
    rclcpp::shutdown();
    return 0;
}