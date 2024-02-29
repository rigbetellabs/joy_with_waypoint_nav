#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <actionlib_msgs/GoalID.h>
#include <std_msgs/Int32.h>
class TeleopHoverboard
{
public:
  TeleopHoverboard();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &odom);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  bool button7_pressed_last_time = false;
  bool button6_pressed_last_time = false;
  int pid_control_value = 1;
  nav_msgs::Odometry odom_received;
  geometry_msgs::PoseStamped home_pose_, pose_x_, pose_y_;

  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  ros::Publisher pose_pub_;
  ros::Subscriber odom_sub_;
  ros::Publisher goal_cancel_pub_;
  ros::Publisher pid_control_pub_;
};

TeleopHoverboard::TeleopHoverboard() : linear_(1),
                                       angular_(0)
{

  nh_.param("axis_linear", linear_, 1);
  nh_.param("axis_angular", angular_, 0);
  nh_.param("scale_angular", a_scale_, 0.6);
  nh_.param("scale_linear", l_scale_, 0.3);

  pose_x_.header.frame_id = "map";
  pose_y_.header.frame_id = "map";

  home_pose_.header.frame_id = "map";
  home_pose_.pose.position.x = 0.0;
  home_pose_.pose.position.y = 0.0;
  home_pose_.pose.position.z = 0.0;

  home_pose_.pose.orientation.w = 1.0;
  home_pose_.pose.orientation.x = 0.0;
  home_pose_.pose.orientation.y = 0.0;
  home_pose_.pose.orientation.z = 0.0;

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10, true);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
  goal_cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
  pid_control_pub_ = nh_.advertise<std_msgs::Int32>("pid/control", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopHoverboard::joyCallback, this);
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 10, &TeleopHoverboard::odomCallback, this);
}

void TeleopHoverboard::odomCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
  odom_received = *odom;
}

void TeleopHoverboard::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
  if (joy->axes[7] == 1)
  {
    l_scale_ = std::max(0.0, l_scale_ + 0.1);
    ROS_INFO_STREAM("Linear Scale Set to : " << l_scale_);
  }
  if (joy->axes[7] == -1)
  {
    l_scale_ = std::max(0.0, l_scale_ - 0.1);
    ROS_INFO_STREAM("Linear Scale Set to : " << l_scale_);
  }
  if (joy->axes[6] == 1)
  {
    a_scale_ = std::max(0.0, a_scale_ - 0.1);
    ROS_INFO_STREAM("Angular Scale Set to : " << a_scale_);
  }
  if (joy->axes[6] == -1)
  {
    a_scale_ = std::max(0.0, a_scale_ + 0.1);
    ROS_INFO_STREAM("Angular Scale Set to : " << a_scale_);
  }

  if (joy->buttons[0] == 1)
  {
    pose_pub_.publish(home_pose_);
    ROS_INFO_STREAM("publishing home position");
  }

  if (joy->buttons[4] == 1)
  {
    pose_x_.pose = odom_received.pose.pose;
  }
  if (joy->buttons[5] == 1)
  {
    pose_y_.pose = odom_received.pose.pose;
  }

  if (joy->buttons[2] == 1)
  {
    pose_pub_.publish(pose_x_);
  }
  if (joy->buttons[3] == 1)
  {
    pose_pub_.publish(pose_y_);
  }
  if (joy->buttons[1] == 1)
  {
    actionlib_msgs::GoalID cancel_goal_;
    goal_cancel_pub_.publish(cancel_goal_);
  }

  if (joy->buttons[7] == 1 && !button7_pressed_last_time)
  {
    // Increment PID control value only when button 7 is pressed for the first time
    pid_control_value++;
    if (pid_control_value == 4)
    {
      pid_control_value = 1;
    }
    button7_pressed_last_time = true;
    ROS_INFO_STREAM("PID Control Value Set to: " << pid_control_value);
    std_msgs::Int32 pid_control_msg;
    pid_control_msg.data = pid_control_value;
    pid_control_pub_.publish(pid_control_msg);
  }
  else if (joy->buttons[7] == 0 && button7_pressed_last_time)
  {
    // Reset the flag when button 7 is released
    button7_pressed_last_time = false;
  }
  if (joy->buttons[6] == 1 && !button6_pressed_last_time)
  {
    pid_control_value = 0;
    button6_pressed_last_time = true;
    ROS_INFO_STREAM("PID Control Disabled ");
    std_msgs::Int32 pid_control_msg;
    pid_control_msg.data = pid_control_value;
    pid_control_pub_.publish(pid_control_msg);
  }
  else if (joy->buttons[6] == 0 && button6_pressed_last_time)
  {
    // Reset the flag when button 7 is released
    button6_pressed_last_time = false;
  }
  if (joy->axes[2] < 0.0)
  {
    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_ * joy->axes[angular_];
    twist.linear.x = l_scale_ * joy->axes[linear_];
    twist.linear.y = l_scale_ * joy->axes[3];
    vel_pub_.publish(twist);
    // ROS_INFO_STREAM("Twist Sent : \n"
    //                 << twist);
  }
  else
  {
    geometry_msgs::Twist twist;
    twist.angular.z = 0.0;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    vel_pub_.publish(twist);
    // ROS_INFO_STREAM("Twist Sent : \n"
    //                 << twist);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop_Hoverboard");

  TeleopHoverboard teleop_Hoverboard;

  ros::spin();
}
