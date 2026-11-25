

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan


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





