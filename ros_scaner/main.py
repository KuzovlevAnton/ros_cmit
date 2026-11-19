#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
# from turtlesim.msg import Pose
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
import math

def pose_callback(msg): # получение позиции (x,y,theta)
    global x,y,theta
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    roll, pitch, yaw = euler_from_quaternion(orientation_list)
    theta=yaw
    # print(x,y)


def scan_callback(msg):
    global scan_data
    scan_data=msg
    print(msg)





x=0 # сброс переменных
y=0
theta=0
pi=3.14159265358979323846264338327950
scan_data=LaserScan()


cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1) # инициализация
odom_sub = rospy.Subscriber('odom', Odometry, pose_callback)
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
rospy.init_node('ros_scaner')



twist = Twist()
rate = rospy.Rate(10)


while not rospy.is_shutdown():
    # print(scan_data)
    for i,d in enumerate(scan_data.ranges):
        plt.plot(math.cos(scan_data.angle_min+i*scan_data.angle_increment)*d, math.sin(scan_data.angle_min+i*scan_data.angle_increment)*d, 'bo')
        print(math.cos(scan_data.angle_min+i*scan_data.angle_increment)*d, math.sin(scan_data.angle_min+i*scan_data.angle_increment)*d)
    plt.show()
    rate.sleep()

