#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
# from turtlesim.msg import Pose
import matplotlib.pyplot as plt


def pose_callback(msg): # получение позиции (x,y,theta)
    global x,y,theta
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    roll, pitch, yaw = euler_from_quaternion(orientation_list)
    theta=yaw
    print(x,y)


def pid_reg(error, kp, ki, kd): # pid регулятор
    global i,last_error
    p = error*kp
    i += error*ki
    d = (last_error-error)*kd
    last_error=error
    u=p+i+d
    return u

def forward_pid(distance, kp, ki, kd, accuracy=0.00001): # pid регулятор на проезд вперёд
    global x,y,theta,max_velocity,max_acceleration,twist,i,last_error
    start_x=x
    start_y=y
    last_error = 0
    i = 0
    while abs(((start_x-x)**2+(start_y-y)**2)**0.5-distance)>accuracy and not rospy.is_shutdown(): # условие завершения: работает пока нужное расстояние не пройдено и программа не завершена
        twist = Twist()
        error = distance-((start_x-x)**2+(start_y-y)**2)**0.5 # ошибка - оставшееся расстояние
        u = pid_reg(error,kp,ki,kd)
        twist.linear.x = min(u, max_acceleration*(2*(abs(((start_x-x)**2+(start_y-y)**2)**0.5 - distance))/max_acceleration)**0.5+accuracy,
        max_acceleration*(2*(((start_x-x)**2+(start_y-y)**2)**0.5)/max_acceleration)**0.5+accuracy)
        cmd_vel_pub.publish(twist)
        rate.sleep()
    twist = Twist() # остановка
    cmd_vel_pub.publish(twist)

def get_angular_dist(theta1, theta2): # угловое расстояие между точками [-pi, pi)
    global pi
    return (theta2-theta1+pi)%(2*pi)-pi


def turn_by_pid(theta_turn, kp, ki, kd, accuracy=0.00001): # pid регулятор на поворот
    global x,y,theta,max_angular_velocity,max_angular_acceleration,twist,pi,i,last_error
    start_theta=theta
    last_error = 0
    i = 0
    while abs(get_angular_dist(theta, start_theta+theta_turn)) > accuracy and not rospy.is_shutdown(): # условие завершения: работает пока нужное угловое расстояние не пройдено и программа не завершена
        twist = Twist()
        error = get_angular_dist(theta, start_theta+theta_turn) # ошибка - оставшееся угловое расстояние
        u = pid_reg(error,kp,ki,kd)
        twist.angular.z = u
        cmd_vel_pub.publish(twist)
        rate.sleep()
    twist = Twist() # остановка
    cmd_vel_pub.publish(twist)


x=0 # сброс переменных
y=0
theta=0
i = 0
last_error = 0
pi=3.14159265358979323846264338327950
max_acceleration=1

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1) # инициализация
odom_sub = rospy.Subscriber('odom', Odometry, pose_callback)
rospy.init_node('square')

twist = Twist()
rate = rospy.Rate(10)

for _ in range(40): # рисоване квадрата (4 стороны, цикл)
    forward_pid(1, 2, 0, 4, accuracy=0.001)
    turn_by_pid(pi/2, 6, 0, 4, accuracy=0.01) # поворот на pi/2 радиан = 90 градусов


