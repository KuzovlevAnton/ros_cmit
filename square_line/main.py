#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan


def pose_callback(msg):
    global x, y, theta
    x = msg.x
    y = msg.y
    theta = msg.theta
    print(x,y,theta)



def turn_to(theta_target, kp, ki, kd, accuracy = 0.0001):
    global x,y,theta, pi
    last_error = 0
    error_i = 0
    twist = Twist()
    theta_end=theta_target
    # theta_end = atan((y_target-y)/(x_target-x))
    # if x_target<x:
    #     theta_end=(theta)%(pi*2)-pi
    while abs(theta_end - theta) > accuracy and not rospy.is_shutdown():
        twist = Twist()
        error = theta_end - theta
        p = kp*error
        error_i += error
        i = error_i * ki
        d = (last_error-error)*kd
        last_error=error
        u = p+i+d
        twist.angular.z = u
        cmd_vel_pub.publish(twist)
        rate.sleep()
    twist = Twist()
    cmd_vel_pub.publish(twist)



def forward(distance, kp, ki, kd, accuracy = 0.0001):
    global x,y,twist
    x_start, y_start = x, y
    last_error = 0
    error_i = 0
    twist = Twist()
    cmd_vel_pub.publish(twist)
    while (abs(((x-x_start)**2+(y-y_start)**2)**0.5-distance) > accuracy) and not rospy.is_shutdown():
        error = distance - ((x-x_start)**2+(y-y_start)**2)**0.5
        p = kp*error
        error_i += error
        i = error_i * ki
        d = (last_error-error)*kd
        last_error=error
        u = p+i+d
        twist.linear.x = u
        cmd_vel_pub.publish(twist)
        rate.sleep()
    twist = Twist()
    cmd_vel_pub.publish(twist)


# def goto(x_target,y_target):
#     global x,y
#     turn_to(x_target,y_target, 1, 0, 0)
#     forward(((x-x_target)**2+(y-y_target)**2)**0.5, 1, 0, 4)






x, y, theta = 0,0,0

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
pose_sub = rospy.Subscriber('pose', Pose, pose_callback)
rospy.init_node('square')

pi=3.14159265358979323846264338327950
max_velocity=10
max_acceleration=10
max_angular_velocity=pi*10

max_angular_acceleration=pi

rate = rospy.Rate(10)
twist = Twist()

# while not rospy.is_shutdown():
#     # # goto(8, 8)
#     # # goto(2, 8)
#     # # goto(2, 2)
#     # # goto(8, 2)
#     # turn_to(0, 1, 0, 0)
#     # forward(1, 1, 0, 0)
#     # turn_to(pi/2, 1, 0, 0)
#     # forward(1, 1, 0, 0)
#     # turn_to(pi, 1, 0, 0)
#     # forward(1, 1, 0, 0)
#     # turn_to(-pi/2, 1, 0, 0)
#     # forward(1, 1, 0, 0)
#     ...
turn_to(0, 1, 0, 0)
forward(3, 1, 0, 0)
turn_to(pi/2, 1, 0, 0)
forward(1, 1, 0, 0)
turn_to(pi, 1, 0, 0)
forward(6, 1, 0, 0)
turn_to(pi/2, 1, 0, 0)
forward(1, 1, 0, 0)
turn_to(0, 1, 0, 0)
forward(6, 1, 0, 0)
turn_to(pi/2, 1, 0, 0)
forward(1, 1, 0, 0)
turn_to(0, 1, 0, 0)
forward(6, 1, 0, 0)
turn_to(pi/2, 1, 0, 0)
forward(1, 1, 0, 0)
turn_to(0, 1, 0, 0)
forward(5, 1, 0, 0)
turn_to(-pi/2, 1, 0, 0)
forward(6, 1, 0, 0)
turn_to(0, 1, 0, 0)
forward(1, 1, 0, 0)
turn_to(pi/2, 1, 0, 0)
forward(6, 1, 0, 0)
turn_to(0, 1, 0, 0)
forward(1, 1, 0, 0)
turn_to(-pi/2, 1, 0, 0)
forward(6, 1, 0, 0)
turn_to(0, 1, 0, 0)
forward(1, 1, 0, 0)
turn_to(pi/2, 1, 0, 0)
forward(6, 1, 0, 0)
turn_to(0, 1, 0, 0)
forward(1, 1, 0, 0)