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
#include "sensor_msgs/msg/joy_feedback.hpp"
#include "rclcpp/time_source.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "std_msgs/msg/int32.hpp"

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
        rumble_pub_ = this->create_publisher<sensor_msgs::msg::JoyFeedback>("/joy/set_feedback", 10);
        pid_pub_ = this->create_publisher<std_msgs::msg::Int32>("pid/control", 10);
        cancel_goal_client_ = this->create_client<action_msgs::srv::CancelGoal>("/navigate_to_pose/_action/cancel_goal");
        clear_costmap_client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("/local_costmap/clear_entirely_local_costmap");
        timer_ = this->create_wall_timer(100ms, std::bind(&AutoJoyTeleop::master_callback, this));
        rumble_timer_ = this->create_wall_timer(100ms, std::bind(&AutoJoyTeleop::rumble_callback, this)); // Lower period than 100ms wont lead to any significant effect

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        a_scale_ = 0.5;
        l_scale_ = 0.5;

        increment_ = 0.01;

        goal_status_ = GoalStatus::NONE;

        rumble_clear_costmap_ = 0;
        rumble_cancel_goal_ = 0;

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

        x_goal_set_ = false;
        y_goal_set_ = false;

        rumble_.type = sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE;
        rumble_.id = 0;

        RCLCPP_INFO(this->get_logger(), "[NODE INITIATED]");
    }

private:
    void master_callback()
    {
        // log_info();
        if (trigger_){
        auto robot_vel = geometry_msgs::msg::Twist();
        robot_vel.linear.x = x_vel_;
        robot_vel.linear.y = y_vel_;
        robot_vel.linear.z = 0.0;
        robot_vel.angular.x = 0.0;
        robot_vel.angular.y = 0.0;
        robot_vel.angular.z = z_vel_;

        cmd_vel_pub_->publish(robot_vel);
        }
    }

    void joy_callback(const sensor_msgs::msg::Joy &joy_msg)
    {
        int x_axis = 1;
        int y_axis = 3;
        int z_axis = 0;
        int l_inc = 7;
        int a_inc = 6;

        l_scale_ = l_scale_ >= 0.0 ? l_scale_ + joy_msg.axes[l_inc] * increment_ : 0.0;
        a_scale_ = a_scale_ >= 0.0 ? a_scale_ + joy_msg.axes[a_inc] * increment_ : 0.0;
        trigger_ =  joy_msg.axes[2] < 0.0 ;
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

            if (!cancel_goal_client_->service_is_ready())
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Cancel Goal Service not available");
            }
            else
            {
                cancelled_goal_ = true;
                auto result = cancel_goal_client_->async_send_request(request);
                goal_status_ = GoalStatus::NONE;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Cancelling Current Goal");
            }
        }
        else if (joy_msg.buttons[2] && goal_status_ != GoalStatus::X)
        {
            if (x_goal_set_)
            {
                goal_pub_->publish(x_goal_);
                goal_status_ = GoalStatus::X;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Setting Robot Goal: X");
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Store X goal before send goal");
            }
        }
        else if (joy_msg.buttons[3] && goal_status_ != GoalStatus::Y)
        {
            if (y_goal_set_)
            {
                goal_pub_->publish(y_goal_);
                goal_status_ = GoalStatus::Y;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Setting Robot Goal: Y");
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Store Y goal before send goal");
            }
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

                x_goal_set_ = true;
                xy_goal_ = true;

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

                y_goal_set_ = true;
                xy_goal_ = true;

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Stored Y pose: (%f, %f, %f)", y_goal_.pose.position.x, y_goal_.pose.position.y, yaw);
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Could not transform %s to %s: %s", "base_link", "map", ex.what());
            }
        }

        if (joy_msg.buttons[8])
        {
            auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();

            if (!clear_costmap_client_->service_is_ready())
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Clear Costmap Service not available");
            }
            else
            {
                cleared_costmap_ = true;
                auto result = clear_costmap_client_->async_send_request(request);
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "CostMap cleared");
            }
        }

        if (joy_msg.buttons[6])
        {
            pid_status_.data = 0;
            pid_pub_->publish(pid_status_);
        }

        if (joy_msg.buttons[7] && !pid_button_pressed_)
        {
            pid_button_pressed_ = true;
            pid_status_.data = ((pid_status_.data + 1) % 3) + 1;
            pid_pub_->publish(pid_status_);
        }
        else if (joy_msg.buttons[7] == 0 && pid_button_pressed_)
        {
            pid_button_pressed_ = false;
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

    void rumble_callback()
    {
        if (cleared_costmap_)
        {
            rumble_clear_costmap_++;
            rumble_.intensity = rumble_clear_costmap_ % 2;
            if (rumble_clear_costmap_ > 7)
            {
                rumble_clear_costmap_ = 0;
                cleared_costmap_ = false;
            }
        }

        if (cancelled_goal_)
        {
            rumble_cancel_goal_++;
            rumble_.intensity = (rumble_cancel_goal_ / 8) ^ 1;
            if (rumble_cancel_goal_ > 7)
            {
                rumble_cancel_goal_ = 0;
                cancelled_goal_ = false;
            }
        }

        if (xy_goal_)
        {
            rumble_xy_goal_++;
            rumble_.intensity = ((rumble_xy_goal_ % 6) < 3) ? 1 : 0;

            if (rumble_xy_goal_ > 9)
            {
                rumble_xy_goal_ = 0;
                xy_goal_ = false;
            }

        }

        rumble_pub_->publish(rumble_);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr rumble_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pid_pub_;
    rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr cancel_goal_client_;
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_costmap_client_;
    rclcpp::TimerBase::SharedPtr rumble_timer_;
    rclcpp::TimerBase::SharedPtr timer_;

    float a_scale_;
    float l_scale_;
    float x_vel_;
    float y_vel_;
    float z_vel_;
    bool trigger_;
    float increment_;

    bool x_goal_set_;
    bool y_goal_set_;

    int log_interval_;
    bool pid_button_pressed_;

    bool cleared_costmap_;
    bool cancelled_goal_;
    bool xy_goal_;

    uint8_t rumble_clear_costmap_;
    uint8_t rumble_cancel_goal_;
    uint8_t rumble_xy_goal_;

    GoalStatus goal_status_;

    geometry_msgs::msg::PoseStamped home_;
    geometry_msgs::msg::PoseStamped x_goal_;
    geometry_msgs::msg::PoseStamped y_goal_;

    sensor_msgs::msg::JoyFeedback rumble_;

    std_msgs::msg::Int32 pid_status_;

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