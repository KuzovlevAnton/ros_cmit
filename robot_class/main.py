#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import math








class Robot:
    pi=3.14159265358979323846264338327950

    def __init__(self):

        self.x=0
        self.y=0
        self.theta=0
        self.max_velocity=3
        self.max_acceleration=2
        self.max_angular_velocity=self.pi*10
        self.max_angular_acceleration=self.pi

        rospy.init_node('robot') # инициализация
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.pose_callback)
        self.scan_sub = rospy.Subscriber('laser/scan', LaserScan, self.scan_callback)

        self.twist = Twist()
        self.rate = rospy.Rate(10)


    def pose_callback(self, msg): # получение позиции (x,y,theta)
        self.x=msg.pose.pose.position.x
        self.y=msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        self.theta=yaw
        print(self.x,self.y)


    def scan_callback(self, msg):
        self.scan_data=msg
        if self.scan_data is None:
            self.scan_data = LaserScan




    def turn(self, theta_turn, accuracy=0.001): # + против часовой стрелки, - по
        start_theta=self.theta
        while abs(start_theta-self.theta) < theta_turn and not rospy.is_shutdown():
            if theta_turn > 0:
                self.twist.angular.z = min(self.max_angular_velocity,
                self.max_angular_acceleration*(2*(abs(start_theta-self.theta))/self.max_angular_acceleration)**0.5+accuracy,
                self.max_angular_acceleration*(2*(abs(theta_turn-abs(start_theta-self.theta)))/self.max_angular_acceleration)**0.5+accuracy)
            if theta_turn < 0:
                self.twist.angular.z = -min(self.max_angular_velocity,
                self.max_angular_acceleration*(2*(abs(start_theta-self.theta))/self.max_angular_acceleration)**0.5+accuracy,
                self.max_angular_acceleration*(2*(abs(theta_turn-abs(start_theta-self.theta)))/self.max_angular_acceleration)**0.5+accuracy)
            self.cmd_vel_pub.publish(self.twist)
            self.rate.sleep()
        self.twist = Twist()
        self.cmd_vel_pub.publish(self.twist)

    class PID:
        def __init__(self, kp, ki, kd, i_border=float('inf')):
            self.i=0
            self.last_error=0
            self.kp=kp
            self.ki=ki
            self.kd=kd
            self.i_border=i_border

        def calculate(self, error):
            p=error*self.kp
            self.i+=error*self.ki
            if self.i > self.i_border:
                self.i = self.i_border
            if self.i < -self.i_border:
                self.i = -self.i_border
            d=(error-self.last_error)*self.kd
            self.last_error=error
            u=p+self.i+d
            print(self.i)
            return u

    def forward(self, distance, kp, ki, kd, accuracy=0.001, i_border=float('inf')): # pid регулятор на проезд вперёд
        start_x=self.x
        start_y=self.y

        def max_accelerated_velocity(distance):
            return (2*self.max_acceleration*distance)**0.5

        pid=self.PID(kp,ki,kd, i_border=i_border)
        while abs(((start_x-self.x)**2+(start_y-self.y)**2)**0.5-distance)>accuracy and not rospy.is_shutdown(): # условие завершения: работает пока нужное расстояние не пройдено и программа не завершена
            max_vel_current=min(self.max_velocity, max_accelerated_velocity(((start_x-self.x)**2+(start_y-self.y)**2)**0.5)+accuracy,
                                max_accelerated_velocity(abs(((start_x-self.x)**2+(start_y-self.y)**2)**0.5-distance))+accuracy)
            twist = Twist()
            error = distance-((start_x-self.x)**2+(start_y-self.y)**2)**0.5 # ошибка - оставшееся расстояние
            u = pid.calculate(error)
            twist.linear.x = max(-max_vel_current, min(max_vel_current, u))
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()
        twist = Twist() # остановка
        self.cmd_vel_pub.publish(twist)



# def get_angular_dist(theta1, theta2): # угловое расстояие между точками [-pi, pi)
#     global pi
#     return (theta2-theta1+pi)%(2*pi)-pi


# def turn_by_pid(theta_turn, kp, ki, kd, accuracy=0.00001): # pid регулятор на поворот
#     global x,y,theta,max_angular_velocity,max_angular_acceleration,twist,pi,i,last_error
#     start_theta=theta
#     last_error = 0
#     i = 0
#     while abs(get_angular_dist(theta, start_theta+theta_turn)) > accuracy and not rospy.is_shutdown(): # условие завершения: работает пока нужное угловое расстояние не пройдено и программа не завершена
#         twist = Twist()
#         error = get_angular_dist(theta, start_theta+theta_turn) # ошибка - оставшееся угловое расстояние
#         u = pid_reg(error,kp,ki,kd)
#         twist.angular.z = u
#         cmd_vel_pub.publish(twist)
#         rate.sleep()
#     twist = Twist() # остановка
#     cmd_vel_pub.publish(twist)



    def scan_all(self):
        return self.scan_data.ranges


    def _scan_range_raw(self, start_angle, end_angle): # angle_min <= start_angle < end_angle <= angle_max
        if start_angle < self.scan_data.angle_min: start_angle=self.scan_data.angle_min
        if end_angle > self.scan_data.angle_max: end_angle = self.scan_data.angle_max
        scan_list=[]
        for i in range(math.ceil((start_angle-self.scan_data.angle_min)/self.scan_data.angle_increment),
                       math.floor((end_angle-self.scan_data.angle_min)/self.scan_data.angle_increment)+1):
            scan_list.append(self.scan_data.ranges[i])
        return scan_list


    def scan_range(self, start_angle, end_angle): # start_angle < end_angle
        if abs(end_angle - start_angle) >= 2*self.pi:
            return self.scan_all()
        if start_angle == end_angle:
            if (start_angle-self.scan_data.angle_min)%self.scan_data.angle_increment <= 0.0001:
                return [self.scan_closest(start_angle)]
            else:
                return list()
        if start_angle > end_angle:
            start_angle, end_angle = end_angle, start_angle
        if self.scan_data.angle_min <= start_angle < end_angle <= self.scan_data.angle_max:
            return self._scan_range_raw(start_angle, end_angle)
        elif start_angle < self.scan_data.angle_min and end_angle > self.scan_data.angle_max:
            return self.scan_all()
        elif start_angle < self.scan_data.angle_min < end_angle and start_angle+2*self.pi < self.scan_data.angle_max:
            return self._scan_range_raw(start_angle+2*self.pi, self.scan_data.angle_max) + self._scan_range_raw(self.scan_data.angle_min, end_angle)
        elif start_angle < self.scan_data.angle_max < end_angle and end_angle-2*self.pi > self.scan_data.angle_min:
            return self._scan_range_raw(start_angle, self.scan_data.angle_max) + self._scan_range_raw(self.scan_data.angle_min, end_angle-2*self.pi)
        return list()

    def scan_closest(self, angle):
        return self.scan_data.ranges[round((angle-self.scan_data.angle_min)/self.scan_data.angle_increment)]


    def scan_xy(self, limit=0):
        scan_list=[]
        if limit == 0: limit = float('inf')
        for i,d in enumerate(self.scan_data.ranges):
            if not math.isnan(d):
                if d <= limit:
                    x,y=math.cos(self.scan_data.angle_min+i*self.scan_data.angle_increment)*d, math.sin(self.scan_data.angle_min+i*self.scan_data.angle_increment)*d
                    scan_list.append((x,y))
        return scan_list




if __name__ == "__main__":
    bot = Robot()
    rospy.wait_for_message('/odom', Odometry, timeout=10)
    # bot.forward(10, 4, 0.001, 1, i_border=0.015)


