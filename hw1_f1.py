#!/usr/bin/env python
from ctypes import alignment
import turtle
from turtlesim.srv import Spawn
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

from g2g_v2 import TurtleBot_Dee

x = 0
y = 0
yaw = 0


def euclidean_distance(goal_pose,current):
        return math.sqrt(pow((current[0] - goal_pose[0]), 2) + pow((current[1] - goal_pose[1]), 2))


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


def move():
    
    vel_msg = Twist()

    #Receiveing the user's input
    print("Let's move your robot")
    speed = float(input("Input your speed:"))
    distance = float(input("Type your distance:"))
    isForward = input("Foward?: ")#True or False

    #Checking if the movement is forward or backwards
    if(isForward):
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = -abs(speed)
    #Since we are moving just in x-axis
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    

    #Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    #Loop to move the turtle in an specified distance
    while(current_distance < distance):
        #Publish the velocity
        velocity_publisher.publish(vel_msg)
        #Takes actual time to velocity calculus
        t1=rospy.Time.now().to_sec()
        #Calculates distancePoseStamped
        current_distance= speed*(t1-t0)
    #After the loop, stops the robot
    vel_msg.linear.x = 0
    #Force the robot to stop
    velocity_publisher.publish(vel_msg)

def rotate_custom(goal_angle,angle_tolerance):

    global yaw

    vel_msg = Twist()

    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    
    while(True):
        #rospy.Subscriber('/turtle1/pose', Pose, poseCallback)
        
        current_angle = yaw

        offset = min(abs(goal_angle - current_angle),abs(2*math.pi - abs(goal_angle - current_angle)))
        rospy.loginfo("Offset angle: %f", offset)

        if offset <= angle_tolerance:
            angular_speed = 0
            break
        else:
            angular_speed = 1

        vel_msg.angular.z = angular_speed

        velocity_publisher.publish(vel_msg)


    vel_msg.linear.x = angular_speed

    velocity_publisher.publish(vel_msg)
        





def rotate (angular_speed_degree, relative_angle_degree, clockwise):
    
    global yaw
    velocity_message = Twist()
    velocity_message.linear.x=0
    velocity_message.linear.y=0
    velocity_message.linear.z=0
    velocity_message.angular.x=0
    velocity_message.angular.y=0
    velocity_message.angular.z=0

    #get current location 
    theta0=yaw
    angular_speed=math.radians(abs(angular_speed_degree))

    if (clockwise):
        velocity_message.angular.z =-abs(angular_speed)
    else:
        velocity_message.angular.z =abs(angular_speed)

    angle_moved = 0.0
    loop_rate = rospy.Rate(50) # we publish the velocity at 10 Hz (10 times a second)    
    cmd_vel_topic='/turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    t0 = rospy.Time.now().to_sec()

    while True :
        
        velocity_publisher.publish(velocity_message)

        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1-t0)*angular_speed_degree
        rospy.loginfo("Offset angle: %f", current_angle_degree)
        loop_rate.sleep()


                       
        if  (current_angle_degree>relative_angle_degree):
            rospy.loginfo("reached")
            break

    #finally, stop the robot when the distance is moved
    velocity_message.angular.z =0
    velocity_publisher.publish(velocity_message)

def go_home(x_goal, y_goal, distance_tolerance):

    global x
    global y
    global yaw

    vel_msg = Twist()

    while euclidean_distance([x_goal,y_goal],[x,y]) >= distance_tolerance:
        
        rospy.Subscriber('/turtle1/pose', Pose, poseCallback)

        vel_msg.linear.x = 1
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 20

        velocity_publisher.publish(vel_msg)
    
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

def align_turtle():

    global yaw

    vel_msg = Twist()

    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    while True:
        rospy.Subscriber('/turtle1/pose', Pose, poseCallback)
        if abs(yaw) > 0.1:
            vel_msg.angular.z = -yaw
            velocity_publisher.publish(vel_msg)
        else:
            break
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
    # Starts a new node
        rospy.init_node('turtlesim_v1', anonymous=True)
        velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        distance_tolerance = 0.01
        angle_tolerance = 0.01


        start_x= float(input("Input your x : "))
        start_y= float(input("Input your y : "))

        TheTurtleBot_Dee = TurtleBot_Dee()

        TheTurtleBot_Dee.move2goal(start_x, start_y,distance_tolerance)
        
        rospy.Subscriber('/turtle1/pose', Pose, poseCallback)

        #rotate(30,130,False)
        #align_turtle()

    
        a = float(input("Input your a : "))
        b = float(input("Input your b : "))
        
        

        
        
        goal_1_cord = [a+x,y]


        

        move_custom(goal_1_cord,distance_tolerance)

        rospy.Subscriber('/turtle1/pose', Pose, poseCallback)

        rotate(20,90,False)
        #rotate_custom(90,angle_tolerance)
        


    except rospy.ROSInterruptException: pass