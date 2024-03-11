#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JoyFeedback, JoyFeedbackArray
from geometry_msgs.msg import Twist

class JoyFeedbackPublisher:
    def __init__(self):
        rospy.init_node('joy_feedback_publisher', anonymous=True)

        # Publisher for joy feedback
        self.joy_feedback_pub = rospy.Publisher('/joy/set_feedback', JoyFeedbackArray, queue_size=1)

        # Subscriber for cmd_vel
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

    def cmd_vel_callback(self, msg):
        # Assuming you want to publish a simple haptic feedback whenever cmd_vel is received
        feedback = JoyFeedback()
        feedback.type = JoyFeedback.TYPE_RUMBLE
        feedback.intensity = 1.0  # You can adjust the intensity as needed

        # Create JoyFeedbackArray and publish
        feedback_array = JoyFeedbackArray()
        feedback_array.array.append(feedback)
        self.joy_feedback_pub.publish(feedback_array)
        rospy.sleep(0.4)
        feedback.intensity = 0.0 
        self.joy_feedback_pub.publish(feedback_array)
        

if __name__ == '__main__':
    try:
        joy_feedback_publisher = JoyFeedbackPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
