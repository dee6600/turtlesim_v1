#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, sqrt, atan2, pi

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

        offset = min(abs(goal_angle - current_angle),abs(2*pi - abs(goal_angle - current_angle)))
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
        





def rotate():
    #Starts a new node
    rospy.init_node('robot_cleaner', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    # Receiveing the user's input
    print("Let's rotate your robot")
    speed = float(input("Input your speed (degrees/sec):"))
    angle = float(input("Type your distance (degrees):"))
    clockwise = input("Clockwise?: ") #True or false

    #Converting from angles to radians
    angular_speed = speed*2*PI/360
    relative_angle = angle*2*PI/360

    #We wont use linear components
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < relative_angle):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)


    #Forcing our robot to stop
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

def go_home(x_goal, y_goal):
    global x
    global y, yaw

    velocity_message = Twist()
    cmd_vel_topic='/turtle1/cmd_vel'

    while (True):
        K_linear = 0.5 
        distance = abs(sqrt(((x_goal-x) ** 2) + ((y_goal-y) ** 2)))

        linear_speed = distance * K_linear


        K_angular = 4.0
        desired_angle_goal = atan2(y_goal-y, x_goal-x)
        angular_speed = (desired_angle_goal-yaw)*K_angular

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed

        velocity_publisher.publish(velocity_message)


        if (distance <0.01):
            break

if __name__ == '__main__':
    try:
    # Starts a new node
        rospy.init_node('turtlesim_v1', anonymous=True)
        velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # start_x= float(input("Input your x : "))
        # start_y= float(input("Input your y : "))
        #home_cord = [start_x, start_y]
        rospy.Subscriber('/turtle1/pose', Pose, poseCallback)

        a = float(input("Input your a : "))
        b = float(input("Input your b : "))

        
        goal_1_cord = [a+x,y]

        
        # goal_3_cord = [0,b]
        # goal_4_cord = [0,2*b]
        # goal_5_cord = [a,2*b]

        distance_tolerance = 0.01
        angle_tolerance = 0.01

        move_custom(goal_1_cord,distance_tolerance)

        rospy.Subscriber('/turtle1/pose', Pose, poseCallback)
        goal_2_cord = [a+x,b+y]

        rotate_custom(90,angle_tolerance)
        


    except rospy.ROSInterruptException: pass