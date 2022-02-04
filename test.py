#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, sqrt

x = 0
y = 0
yaw = 0


def euclidean_distance(goal_pose,current):
        return sqrt(pow((current[0] - goal_pose[0]), 2) + pow((current[1] - goal_pose[1]), 2))


def poseCallback(data):
    global x
    global y
    global yaw

    x = data.x
    y = data.y
    yaw = data.theta



def move_custom(goal_cord, distance_tolerance):

    global x
    global y

    vel_msg = Twist()

    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    while(True):

        rospy.Subscriber('/turtle1/pose', Pose, poseCallback)
        
        current_cord = [x,y]

        distance = euclidean_distance(goal_cord, current_cord)

        rospy.loginfo("Distance to goal: %f", distance)

        if distance <= distance_tolerance:
            linear_speed = 0
            break
        else:
            linear_speed = 1

        vel_msg.linear.x = linear_speed

        velocity_publisher.publish(vel_msg)


    vel_msg.linear.x = linear_speed

    velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    try:
    # Starts a new node
        rospy.init_node('turtlesim_v1', anonymous=True)
        velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)


        

        pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, poseCallback)

        a = float(input("Input your a : "))
        goal_1_cord = [a+x,y]

        distance_tolerance = 0.01

        move_custom(goal_1_cord,distance_tolerance)

    except rospy.ROSInterruptException: pass
