#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JoyFeedbackArray.h>
#include <std_srvs/Empty.h>

#include <tf/transform_datatypes.h>

using namespace std;

class AutoJoyTeleop
{
public:
  AutoJoyTeleop()
  {
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10, true);
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    goal_cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
    rumble_pub_ = nh_.advertise<sensor_msgs::JoyFeedbackArray>("/joy/set_feedback", 10);
    pid_pub_ = nh_.advertise<std_msgs::Int32>("pid/control", 1);
    nav_status_pub_ = nh_.advertise<std_msgs::Int32>("robot/nav_status", 1);

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &AutoJoyTeleop::joyCallback, this);
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 10, &AutoJoyTeleop::odomCallback, this);

    timer_ = nh_.createWallTimer(ros::WallDuration(1.0), &AutoJoyTeleop::timerCallback, this); // Duration between adjacent publishment should be atleast 1sec

    a_scale_ = 0.5;
    l_scale_ = 0.5;
    x_vel_ = 0.0;
    y_vel_ = 0.0;
    z_vel_ = 0.0;
    increment_ = 0.01;
    trigger_ = false;

    pid_value_ = 0;
    pid_button_pressed_ = false;

    rumble_clear_costmap_ = 0;
    rumble_cancel_goal_ = 0;
    rumble_xy_goal_ = 0;
    cancelled_goal_ = false;
    xy_goal_ = false;
    cleared_costmap_ = false;
    rumble_.type = sensor_msgs::JoyFeedback::TYPE_RUMBLE;
    rumble_.id = 0;

    x_goal_set_ = false;
    y_goal_set_ = false;

    home_.header.frame_id = "map";
    home_.header.stamp = ros::Time::now();
    home_.pose.position.x = 0.0;
    home_.pose.position.y = 0.0;
    home_.pose.position.z = 0.0;
    home_.pose.orientation.x = 0.0;
    home_.pose.orientation.y = 0.0;
    home_.pose.orientation.z = 0.0;
    home_.pose.orientation.w = 1.0;

    ROS_INFO("[NODE INITIATED]");

  }

private:

  void timerCallback(const ros::WallTimerEvent&)
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

    sensor_msgs::JoyFeedbackArray rumble_array_;

    rumble_array_.array.push_back(rumble_);

    rumble_pub_.publish(rumble_array_);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &odom)
  {
    robot_pose = *odom;
  }

  void joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg)
  {
    /* ------------------------------------ TELEOP ------------------------------------ */
    int x_axis = 1;
    int y_axis = 3;
    int z_axis = 0;
    int l_inc = 7;
    int a_inc = 6;

    l_scale_ = l_scale_ >= 0.0 ? l_scale_ + joy_msg->axes[l_inc] * increment_ : 0.0;
    a_scale_ = a_scale_ >= 0.0 ? a_scale_ - joy_msg->axes[a_inc] * increment_ : 0.0;

    x_vel_ = joy_msg->axes[2] < 0.0 ? l_scale_ * joy_msg->axes[x_axis] : 0.0;
    y_vel_ = joy_msg->axes[2] < 0.0 ? l_scale_ * joy_msg->axes[y_axis] : 0.0;
    z_vel_ = joy_msg->axes[2] < 0.0 ? a_scale_ * joy_msg->axes[z_axis] : 0.0;

    if (joy_msg->axes[2] < 1.0)
    {
      geometry_msgs::Twist robot_vel;
      robot_vel.linear.x = x_vel_;
      robot_vel.linear.y = y_vel_;
      robot_vel.linear.z = 0.0;
      robot_vel.angular.x = 0.0;
      robot_vel.angular.y = 0.0;
      robot_vel.angular.z = z_vel_;

      trigger_ = true;

      cmd_vel_pub_.publish(robot_vel);
    }
    else
    {
      if (trigger_)
      {
        geometry_msgs::Twist robot_vel;
        robot_vel.linear.x = 0.0;
        robot_vel.linear.y = 0.0;
        robot_vel.linear.z = 0.0;
        robot_vel.angular.x = 0.0;
        robot_vel.angular.y = 0.0;
        robot_vel.angular.z = 0.0;

        trigger_ = false;

        cmd_vel_pub_.publish(robot_vel);

      }
    }

    /* ------------------------------------ PID ------------------------------------ */

    if (joy_msg->buttons[6])
    {
      pid_value_ = 0;
      pid_status_.data = pid_value_;

      ROS_INFO("PID status: 0");
      pid_pub_.publish(pid_status_);
    }


    if (joy_msg->buttons[7] && !pid_button_pressed_)
    {
      pid_button_pressed_ = true;
      pid_status_.data = (pid_value_ % 3) + 1;
      pid_value_++;
      ROS_INFO("PID status: %d", pid_status_.data);
      pid_pub_.publish(pid_status_);
    }
    else if (joy_msg->buttons[7] == 0 && pid_button_pressed_)
    {
      pid_button_pressed_ = false;
    }

    /* ------------------------------------ COSTMAP ------------------------------------ */

    if (joy_msg->buttons[8])
    {
      ros::ServiceClient clear_costmap_client = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

      if (clear_costmap_client.exists())
      {
        std_srvs::Empty empty_msg;
        clear_costmap_client.call(empty_msg);

        cleared_costmap_ = true;

        ROS_INFO("CostMap cleared");
      }
      else
      {
        ROS_WARN("Clear Costmap Service not available");
      }
    }


    /* ------------------------------------ STORE POSE ------------------------------------ */

    if (joy_msg->buttons[4])
    {
      x_goal_.header.frame_id = "map";
      x_goal_.header.stamp = ros::Time::now();
      x_goal_.pose.position.x = robot_pose.pose.pose.position.x;
      x_goal_.pose.position.y = robot_pose.pose.pose.position.y;
      x_goal_.pose.position.z = 0.0;
      x_goal_.pose.orientation.x = 0.0;
      x_goal_.pose.orientation.y = 0.0;
      x_goal_.pose.orientation.z = robot_pose.pose.pose.orientation.z;
      x_goal_.pose.orientation.w = robot_pose.pose.pose.orientation.w;

      double roll, pitch, yaw;
      tf::Quaternion quats(x_goal_.pose.orientation.x, x_goal_.pose.orientation.y, x_goal_.pose.orientation.z, x_goal_.pose.orientation.w);
      tf::Matrix3x3(quats).getRPY(roll, pitch, yaw);

      x_goal_set_ = true;

      ROS_INFO("Stored X pose: (%f, %f, %f)", x_goal_.pose.position.x, x_goal_.pose.position.y, yaw);
    }

    if (joy_msg->buttons[5])
    {
      y_goal_.header.frame_id = "map";
      y_goal_.header.stamp = ros::Time::now();
      y_goal_.pose.position.x = robot_pose.pose.pose.position.x;
      y_goal_.pose.position.y = robot_pose.pose.pose.position.y;
      y_goal_.pose.position.z = 0.0;
      y_goal_.pose.orientation.x = 0.0;
      y_goal_.pose.orientation.y = 0.0;
      y_goal_.pose.orientation.z = robot_pose.pose.pose.orientation.z;
      y_goal_.pose.orientation.w = robot_pose.pose.pose.orientation.w;

      double roll, pitch, yaw;
      tf::Quaternion quats(y_goal_.pose.orientation.x, y_goal_.pose.orientation.y, y_goal_.pose.orientation.z, y_goal_.pose.orientation.w);
      tf::Matrix3x3(quats).getRPY(roll, pitch, yaw);

      y_goal_set_ = true;

      ROS_INFO("Stored Y pose: (%f, %f, %f)", y_goal_.pose.position.x, y_goal_.pose.position.y, yaw);
    }

    /* ------------------------------------ SEND GOAL ------------------------------------ */

    if (joy_msg->buttons[2])
    {
      if (x_goal_set_)
      {
        goal_pub_.publish(x_goal_);

        std_msgs::Int32 goal_status_code;
        goal_status_code.data = 5;
        nav_status_pub_.publish(goal_status_code);

        xy_goal_ = true;

        ROS_INFO("Setting Robot Goal: X");
      }
      else
      {
        ROS_INFO("Store X goal before send goal");
      }
    }

    if (joy_msg->buttons[3])
    {
      if (y_goal_set_)
      {
        goal_pub_.publish(y_goal_);

        std_msgs::Int32 goal_status_code;
        goal_status_code.data = 5;
        nav_status_pub_.publish(goal_status_code);

        xy_goal_ = true;

        ROS_INFO("Setting Robot Goal: Y");
      }
      else
      {
        ROS_INFO("Store Y goal before send goal");
      }
    }

    if (joy_msg->buttons[0])
    {
      goal_pub_.publish(home_);

      std_msgs::Int32 goal_status_code;
      goal_status_code.data = 5;
      nav_status_pub_.publish(goal_status_code);

      ROS_INFO("Setting Robot Goal: HOME");
    }

    /* ------------------------------------ CANCEL GOAL ------------------------------------ */

    if (joy_msg->buttons[1])
    {
      actionlib_msgs::GoalID cancel_goal_;
      goal_cancel_pub_.publish(cancel_goal_);

      std_msgs::Int32 goal_status_code;
      goal_status_code.data = 4;
      nav_status_pub_.publish(goal_status_code);

      cancelled_goal_ = true;

      ROS_INFO_STREAM("Cancelling Current Goal");
    }

    /* ------------------------------------------------------------------------------ */
  }

  ros::NodeHandle nh_;

  ros::Publisher cmd_vel_pub_;
  ros::Publisher goal_pub_;
  ros::Publisher goal_cancel_pub_;
  ros::Publisher rumble_pub_;
  ros::Publisher pid_pub_;
  ros::Publisher nav_status_pub_;

  ros::Subscriber joy_sub_;
  ros::Subscriber odom_sub_;

  ros::WallTimer timer_;

  sensor_msgs::JoyFeedback rumble_;

  /* ------------------------------------ TELEOP ------------------------------------ */

  float a_scale_;
  float l_scale_;
  float x_vel_;
  float y_vel_;
  float z_vel_;
  float increment_;
  bool trigger_;

  /* ------------------------------------ PID ------------------------------------ */

  uint8_t pid_value_;
  std_msgs::Int32 pid_status_;
  bool pid_button_pressed_;


  /* ------------------------------------ COSTMAP ------------------------------------ */

  bool cleared_costmap_;
  uint8_t rumble_clear_costmap_;


  /* ------------------------------------ CANCEL POSE ------------------------------------ */

  bool cancelled_goal_;
  uint8_t rumble_cancel_goal_;

  /* ------------------------------------ STORE POSE ------------------------------------ */

  nav_msgs::Odometry robot_pose;

  geometry_msgs::PoseStamped x_goal_;
  geometry_msgs::PoseStamped y_goal_;

  bool x_goal_set_;
  bool y_goal_set_;

  bool xy_goal_;

  uint8_t rumble_xy_goal_;


  /* ------------------------------------ SEND GOAL ------------------------------------ */

  geometry_msgs::PoseStamped home_;

  /* ------------------------------------------------------------------------------ */

};




int main(int argc, char **argv)
{
  ros::init(argc, argv, "auto_joy_teleop");

  AutoJoyTeleop autojoyteleop;

  ros::spin();
}
