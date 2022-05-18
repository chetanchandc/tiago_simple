#!/usr/bin/env python

import rospy #1
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

#5
x = 0.0
y = 0.0
theta = 0.0

#stores the current psotion of the robot(x,y, theta) #4
def newOdom(msg):
    global x   #6
    global y
    global theta

    x = msg.pose.pose.position.x   #7
    y = msg.pose.pose.position.y
   

#getting theta(is the rotation around the z-axis) value is not same as x and y. So we need euler from quaternion(which is provided by tf)  

    rot_q = msg.pose.pose.orientation #9
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])  #8



rospy.init_node("speed_controller") #2

sub = rospy.Subscriber("/mobile_base_controller/odom", Odometry, newOdom) #3
pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=1) #16

speed = Twist()  #14

r = rospy.Rate(4) #17   For controlling the loop(as it is too fast in rviz)

goal = Point ()   #11
goal.x = 10
goal.y = 10

while not rospy.is_shutdown():  #10  if robot is not facing towards the goal
    inc_x = goal.x - x
    inc_y = goal.y - y
    
                                                    #12 
    angle_to_goal = atan2 (inc_y, inc_x)
                                                    #13
    if abs(angle_to_goal - theta) > 0.1:
        speed.linear.x = 0.0
        speed.angular.z = 0.3
    else:                                             #14
        speed.linear.x = 0.5
        speed.angular.z = 0.0

# Now publish the above values in the publisher         #15

    pub.publish(speed)
    r.sleep()   #18
    rospy.loginfo("goal pose set [5,5]")

