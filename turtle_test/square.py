#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


def forward(distance, accuracy=0.001):
    global x,y,theta,max_velocity,max_acceleration,twist
    start_x=x
    start_y=y
    while ((start_x-x)**2+(start_y-y)**2)**0.5 < distance and not rospy.is_shutdown():
        twist.linear.x = min(max_velocity,
        max_acceleration*(2*(abs(((start_x-x)**2+(start_y-y)**2)**0.5 - distance))/max_acceleration)**0.5+accuracy,
        max_acceleration*(2*(((start_x-x)**2+(start_y-y)**2)**0.5)/max_acceleration)**0.5+accuracy)
        cmd_vel_pub.publish(twist)
        rate.sleep()
    twist = Twist()
    cmd_vel_pub.publish(twist)

def turn_by(theta_turn, accuracy=0.001):
    global x,y,theta,max_angular_velocity,max_angular_acceleration,twist,pi
    start_theta=theta
    while abs(start_theta-theta) < theta_turn and not rospy.is_shutdown():
        if theta_turn > 0:
            twist.angular.z = min(max_angular_velocity,
            max_angular_acceleration*(2*(abs(start_theta-theta))/max_angular_acceleration)**0.5+accuracy,
            max_angular_acceleration*(2*(abs(theta_turn-abs(start_theta-theta)))/max_angular_acceleration)**0.5+accuracy)
        if theta_turn < 0:
            twist.angular.z = -min(max_angular_velocity,
            max_angular_acceleration*(2*(abs(start_theta-theta))/max_angular_acceleration)**0.5+accuracy,
            max_angular_acceleration*(2*(abs(theta_turn-abs(start_theta-theta)))/max_angular_acceleration)**0.5+accuracy)
        cmd_vel_pub.publish(twist)
        rate.sleep()
    twist = Twist()
    cmd_vel_pub.publish(twist)



def pose_callback(msg):
    global x,y,theta
    x=msg.x
    y=msg.y
    theta=msg.theta

x=0
y=0
theta=0
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
pose_sub = rospy.Subscriber('pose', Pose, pose_callback)
rospy.init_node('square')

twist = Twist()
# twist.linear.x = 0.5
# twist.angular.z = 0.5
pi=3.14159265358979323846264338327950
max_velocity=10
max_acceleration=10
max_angular_velocity=pi*10
max_angular_acceleration=pi

rate = rospy.Rate(10)

for _ in range(40):
    forward(3)
    turn_by(pi/2)
# while not rospy.is_shutdown():
#     # cmd_vel_pub.publish(twist)
#     rate.sleep()
