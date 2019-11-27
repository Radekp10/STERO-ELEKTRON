#!/usr/bin/env python

import rospy
from geometry_msgs.msg import *
from stero_mobile_init.srv import *
import math
import time
from nav_msgs.msg import *
from tf.transformations import *
from std_msgs.msg import *


laser_x=0.0
laser_y=0.0
laser_theta=0.0

gazebo_odom_x=0.0
gazebo_odom_y=0.0
gazebo_odom_theta=0.0

diff_x=0.0
diff_y=0.0
diff_theta=0.0


def get_position_laser(data):

    global laser_x 
    global laser_y
    global laser_theta

    laser_x=data.x
    laser_y=data.y
    laser_theta=data.theta

def get_position_gazebo(data):

    global gazebo_odom_x 
    global gazebo_odom_y
    global gazebo_odom_theta
    orientation=[data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
    _, _, theta1 = euler_from_quaternion(orientation)
    #print 'x=',data.pose.pose.position.x, '  y=', data.pose.pose.position.y, '  theta=', theta1
    
    gazebo_odom_x=data.pose.pose.position.x
    gazebo_odom_y=data.pose.pose.position.y
    gazebo_odom_theta=theta1


def collect_data():
	
    global diff_x
    global diff_y
    global diff_theta

    sub_pos_laser = rospy.Subscriber('pose2D', Pose2D, get_position_laser)
    sub_pos_gazebo = rospy.Subscriber('gazebo_odom', Odometry, get_position_gazebo)
    

    pom_odom_x=laser_x
    pom_odom_y=laser_y
    pom_odom_theta=laser_theta
    x=gazebo_odom_x
    y=gazebo_odom_y
    theta=gazebo_odom_theta
    diff_x=pom_odom_x-x
    diff_y=pom_odom_y-y
    diff_theta=pom_odom_theta-theta
    
			

    pub = rospy.Publisher('chatter', Pose2D, queue_size=10)
    pub_diff_x=rospy.Publisher('diff_x', Float64, queue_size=10)
    pub_diff_y=rospy.Publisher('diff_y', Float64, queue_size=10)
    pub_diff_theta=rospy.Publisher('diff_theta', Float64, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
	pose2d=Pose2D()
	pose2d.x=x
	pose2d.y=y
	pose2d.theta=theta
        pub.publish(pose2d)
	pub_diff_x.publish(diff_x)
	pub_diff_y.publish(diff_y)
	pub_diff_theta.publish(diff_theta)
        rate.sleep()

	pom_odom_x=laser_x
	pom_odom_y=laser_y
	pom_odom_theta=laser_theta
	x=gazebo_odom_x
        y=gazebo_odom_y
	theta=gazebo_odom_theta
	diff_x=pom_odom_x-x
	diff_y=pom_odom_y-y
	diff_theta=pom_odom_theta-theta
	
    
    return "Koniec"



if __name__ == "__main__":

    rospy.init_node('Projekt1_chatter')
    collect_data()

    rospy.spin()