#!/usr/bin/env python
from turtle import right
from numpy import MAY_SHARE_EXACT
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, radians, sqrt, sin , cos


class TurtleBot_Dee:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        #rospy.init_node('turtlebot_controller', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
                                                Pose, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(100)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        self.pose.theta = self.pose.theta

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def move2goal(self,my_x,my_y, distance_tolerance):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        # Get the input from the user.
        # goal_pose.x = float(input("Set your x goal: "))
        # goal_pose.y = float(input("Set your y goal: "))
        goal_pose.x = my_x
        goal_pose.y = my_y    
        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        

        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)


    def align_turtle(self, direction):

            vel_msg = Twist()

            if direction == "right":
                while abs(self.pose.theta) >= 0.0001:
                    rospy.loginfo("angle: %f", self.pose.theta)
                    vel_msg.linear.x = 0
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0
                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0
                    vel_msg.angular.z = radians(80) * abs(self.pose.theta)
                    self.velocity_publisher.publish(vel_msg)
                    #self.rate.sleep()
                # Stopping our robot after the movement is over.
                vel_msg.angular.z = 0
                self.velocity_publisher.publish(vel_msg)

            if direction == 'up':
                while abs(self.pose.theta) <= 90.0001:
                    rospy.loginfo("angle: %f", self.pose.theta)
                    vel_msg.linear.x = 0
                    vel_msg.linear.y = 0
                    vel_msg.linear.z = 0
                    vel_msg.angular.x = 0
                    vel_msg.angular.y = 0
                    vel_msg.angular.z = radians(80) * abs(self.pose.theta)
                    self.velocity_publisher.publish(vel_msg)
                    #self.rate.sleep()
                # Stopping our robot after the movement is over.
                vel_msg.angular.z = 0
                self.velocity_publisher.publish(vel_msg) 

    def custom_rotate(self, relative_angle):

        vel_msg = Twist()

        theta0 = self.pose.theta

        angular_speed = 30

        angle_moved = 0

        t0 = rospy.Time.now().to_sec()

        while True:
            t1 = rospy.Time.now().to_sec()
            current_angle = (t1-t0) * angular_speed
            rospy.loginfo("current_angle: %f", current_angle)
            if (current_angle >= relative_angle):
                break

            vel_msg.angular.z = radians(angular_speed)
            self.velocity_publisher.publish(vel_msg)
            

        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)    

    def custom_move(self, distance):

        vel_msg = Twist()

        distance_moved = 0

        x0 = self.pose.x
        y0 = self.pose.y
        
        while True:
            vel_msg.linear.x = 1
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            #self.rate.sleep()
            distance_moved = abs(sqrt(((self.pose.x - x0)**2) + ((self.pose.y - y0)**2)))
            if not (distance_moved < distance):
                break

        vel_msg.linear.x = 0    
        self.velocity_publisher.publish(vel_msg)
        

if __name__ == '__main__':
    try:
        # Initialize the node and name it.
        rospy.init_node('turtlebot_controller', anonymous=True)

        x_start = float(input("Set your x start: "))
        y_start = float(input("Set your y start: "))

        x = TurtleBot_Dee()

        distance_tolerance = 0.01

        x.move2goal(x_start,y_start,distance_tolerance)
        x.align_turtle(direction = "right")

        user_a = float(input("Input a: "))
        user_b = float(input("Input b: "))

        current_pose = Pose()

        point_1 = [x_start + user_a, y_start]
        x.custom_move(user_a)
        #x.move2goal(point_1[0],point_1[1],distance_tolerance)
        x.custom_rotate(90)

        #point_2 = [x_start + user_a, y_start + user_b]
        # x.move2goal(point_2[0],point_2[1],distance_tolerance)
        # x.move2goal(point_3[0],point_3[1],distance_tolerance)
        # x.move2goal(point_4[0],point_4[1],distance_tolerance)
        # x.move2goal(point_5[0],point_5[1],distance_tolerance)

        # x.move2goal(x_start,y_start,distance_tolerance)


    except rospy.ROSInterruptException:
        pass