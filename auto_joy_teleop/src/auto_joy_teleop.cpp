#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JoyFeedbackArray.h>
#include <sensor_msgs/JoyFeedback.h>
#include <std_srvs/Empty.h>
#include <unistd.h>

class TeleopHoverboard
{
public:
  TeleopHoverboard();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &odom);
  sensor_msgs::JoyFeedback createFeedback(double intensity);
  void publishTwist(const sensor_msgs::Joy::ConstPtr &joy);
  void stopTwist();
  void publishHapticFeedback(const sensor_msgs::JoyFeedback &feedback, int duration_ms, int iterations);
  void stopHapticFeedback();
  void goalStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &status);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  bool button7_pressed_last_time = false;
  bool button6_pressed_last_time = false;
  bool button4_pressed_last_time = false;
  bool button5_pressed_last_time = false;
  int pid_control_value = 1;
  bool goal_reached_ = false;
  bool x_pose_stored_ = false;
  bool y_pose_stored_ = false;
  nav_msgs::Odometry odom_received;
  geometry_msgs::PoseStamped home_pose_, pose_x_, pose_y_;

  ros::Publisher vel_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher goal_cancel_pub_;
  ros::Publisher joy_feedback_pub_;
  ros::Publisher pid_control_pub_;
  ros::Publisher goal_status_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber goal_status_sub_;
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
  joy_feedback_pub_ = nh_.advertise<sensor_msgs::JoyFeedbackArray>("/joy/set_feedback", 10);
  pid_control_pub_ = nh_.advertise<std_msgs::Int32>("pid/control", 1);
  goal_status_pub_ = nh_.advertise<std_msgs::Int32>("robot/status", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopHoverboard::joyCallback, this);
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 10, &TeleopHoverboard::odomCallback, this);
  goal_status_sub_ = nh_.subscribe<actionlib_msgs::GoalStatusArray>("move_base/status", 10, &TeleopHoverboard::goalStatusCallback, this);
}

void TeleopHoverboard::odomCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
  odom_received = *odom;
}

sensor_msgs::JoyFeedback TeleopHoverboard::createFeedback(double intensity)
{
  sensor_msgs::JoyFeedback feedback;
  feedback.type = 1;
  feedback.id = 0;
  feedback.intensity = intensity;
  return feedback;
}

void TeleopHoverboard::publishTwist(const sensor_msgs::Joy::ConstPtr &joy)
{
  geometry_msgs::Twist twist;
  twist.angular.z = a_scale_ * joy->axes[angular_];
  twist.linear.x = l_scale_ * joy->axes[linear_];
  twist.linear.y = l_scale_ * joy->axes[3];
  vel_pub_.publish(twist);
}

void TeleopHoverboard::stopTwist()
{
  geometry_msgs::Twist twist;
  twist.angular.z = 0.0;
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  vel_pub_.publish(twist);
}
void TeleopHoverboard::goalStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &status)
{
  // Assuming that you are interested in the first goal status in the array
  if (!status->status_list.empty())
  {
    int goal_status = status->status_list[0].status;
    if (goal_status == actionlib_msgs::GoalStatus::SUCCEEDED && !goal_reached_)
    {
      // Trigger haptic feedback when the goal is reached for the first time
      publishHapticFeedback(createFeedback(1.0), 200, 1);

      // Set the flag to true to prevent continuous triggering
      goal_reached_ = true;
      ROS_INFO_STREAM("Goal Reached Vibration achieved");
    }
    else if (goal_status != actionlib_msgs::GoalStatus::SUCCEEDED)
    {
      // Reset the flag if the goal status is not "SUCCEEDED"
      goal_reached_ = false;
    }
  }
}
void TeleopHoverboard::publishHapticFeedback(const sensor_msgs::JoyFeedback &feedback, int duration_ms, int iterations)
{
  for (int i = 0; i < iterations; i++)
  {
    sensor_msgs::JoyFeedbackArray feedback_array;
    feedback_array.array.push_back(feedback);
    joy_feedback_pub_.publish(feedback_array);
    ros::Duration(duration_ms / 1000.0).sleep(); // Sleep for specified duration in ms
    stopHapticFeedback();
    ros::Duration(duration_ms / 2000.0).sleep();
  }
}

void TeleopHoverboard::stopHapticFeedback()
{
  sensor_msgs::JoyFeedback stop_feedback;
  stop_feedback.type = 1;
  stop_feedback.id = 0;          // Assuming the same ID used for starting feedback
  stop_feedback.intensity = 0.0; // Stop haptic feedback

  sensor_msgs::JoyFeedbackArray feedback_array;
  feedback_array.array.push_back(stop_feedback);
  joy_feedback_pub_.publish(feedback_array);
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
  if (joy->buttons[4] == 1 && !button4_pressed_last_time && !x_pose_stored_)
  {
    // Store pose_x_
    pose_x_.pose = odom_received.pose.pose;
    button4_pressed_last_time = true;
    x_pose_stored_ = true;  // Set the flag
    publishHapticFeedback(createFeedback(1.0), 200, 1);
    ROS_INFO_STREAM("X pose store");
    std_msgs::Int32 goal_status_code;
    goal_status_code.data = 5;
    goal_status_pub_.publish(goal_status_code);
  }
  else if (joy->buttons[4] == 0 && button4_pressed_last_time)
  {
    // Reset the flag when button 7 is released
    button4_pressed_last_time = false;
    x_pose_stored_ = false;  // Reset the flag
  }

  if (joy->buttons[5] == 1 && !button5_pressed_last_time && !y_pose_stored_)
  {
    // Store pose_y_
    pose_y_.pose = odom_received.pose.pose;
    button5_pressed_last_time = true;
    y_pose_stored_ = true;  // Set the flag
    publishHapticFeedback(createFeedback(1.0), 200, 1);
    ROS_INFO_STREAM("Y pose store");
    std_msgs::Int32 goal_status_code;
    goal_status_code.data = 5;
    goal_status_pub_.publish(goal_status_code);
  }
  else if (joy->buttons[5] == 0 && button5_pressed_last_time)
  {
    // Reset the flag when button 7 is released
    button5_pressed_last_time = false;
    y_pose_stored_ = false;  // Reset the flag
  }

  if (joy->buttons[0] == 1)
  {
    pose_pub_.publish(home_pose_);
    ROS_INFO_STREAM("Publishing home position");
  }

  if (joy->buttons[1] == 1)
  {
    // Cancel the goal
    actionlib_msgs::GoalID cancel_goal_;
    goal_cancel_pub_.publish(cancel_goal_);
    ROS_INFO_STREAM("Cancel Goal");

    publishHapticFeedback(createFeedback(1.0), 200, 3);
  }

  if (joy->buttons[2] == 1)
  {
    pose_pub_.publish(pose_x_);
  }
  if (joy->buttons[3] == 1)
  {
    pose_pub_.publish(pose_y_);
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
  if (joy->buttons[8] == 1)
  {
    // Call the service to clear the costmap
    ros::ServiceClient clear_costmap_client = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    std_srvs::Empty empty_msg;
    if (clear_costmap_client.call(empty_msg))
    {
      ROS_INFO("Costmap cleared");
      publishHapticFeedback(createFeedback(1.0), 300, 1);
      std_msgs::Int32 goal_status_code;
      goal_status_code.data = 4;
      goal_status_pub_.publish(goal_status_code);
    }
    else
    {
      ROS_ERROR("Failed to call clear_costmap service");
    }
  }
  if (joy->axes[2] < 0.0)
  {
    // Move based on joystick input
    publishTwist(joy);
  }
  else
  {
    // Stop movement
    stopTwist();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop_Hoverboard");

  TeleopHoverboard teleop_Hoverboard;

  ros::spin();
}
