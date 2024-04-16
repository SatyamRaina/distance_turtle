#!/usr/bin/env python3
# Satyam Raina
# 222196882

# Import Dependencies
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import time
import math

class TurtlesimStraightsAndTurns:
    def __init__(self):
        # Initialize class variables
        self.current_pose = Pose()
        self.goal_distance = 0.0
        self.goal_angle = 0.0
        self.is_distance_goal_active = False
        self.is_angle_goal_active = False
        self.is_moving_forward = True

        # Initialize the node
        rospy.init_node('turtlesim_straights_and_turns_node', anonymous=True)

        # Initialize subscribers
        rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback)
        rospy.Subscriber("/goal_distance", Float64, self.goal_distance_callback)
        rospy.Subscriber("/goal_angle", Float64, self.goal_angle_callback)

        # Initialize publishers
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Initialize a timer. The timer callback will act as our main function
        timer_period = 0.01
        rospy.Timer(rospy.Duration(timer_period), self.timer_callback)

        # Printing to the terminal, ROS style
        rospy.loginfo("Initialized node!")

        # This blocking function call keeps python from exiting until node is stopped
        rospy.spin()

    def pose_callback(self, msg):
        self.current_pose = msg

    def goal_distance_callback(self, msg):
        self.goal_distance = msg.data
        self.is_distance_goal_active = True
        self.is_moving_forward = self.goal_distance >= 0.0

    def goal_angle_callback(self, msg):
        self.goal_angle = msg.data
        self.is_angle_goal_active = True

    def timer_callback(self, msg):
        twist = Twist()

        # Handle distance goal
        if self.is_distance_goal_active:
            distance_error = abs(self.current_pose.x - self.goal_distance)
            if distance_error < 0.1:
                self.is_distance_goal_active = False
                rospy.loginfo("Distance goal reached!")
            else:
                twist.linear.x = 0.5 if self.is_moving_forward else -0.5

        # Handle angle goal
        if self.is_angle_goal_active:
            angle_error = abs(self.current_pose.theta - self.goal_angle)
            if angle_error < 0.1:
                self.is_angle_goal_active = False
                rospy.loginfo("Angle goal reached!")
            else:
                twist.angular.z = 0.5 if self.goal_angle > self.current_pose.theta else -0.5

        # If no goals are active, stop the turtle
        if not self.is_distance_goal_active and not self.is_angle_goal_active:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.velocity_publisher.publish(twist)

if __name__ == '__main__':
    try:
        turtlesim_straights_and_turns_class_instance = TurtlesimStraightsAndTurns()
    except rospy.ROSInterruptException:
        pass
