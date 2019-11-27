#!/usr/bin/env python

import rospy
from geometry_msgs.msg import *
from stero_mobile_init.srv import *
import math
import time
from nav_msgs.msg import *
from tf.transformations import *


odom_x=0.0
odom_y=0.0
odom_theta=0.0

gazebo_odom_x=0.0
gazebo_odom_y=0.0
gazebo_odom_theta=0.0

diff_x=0.0
diff_y=0.0
diff_theta=0.0


def get_position_controller(data):

    global odom_x 
    global odom_y
    global odom_theta
    orientation=[data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
    _, _, theta1 = euler_from_quaternion(orientation)
    #print 'x=',data.pose.pose.position.x, '  y=', data.pose.pose.position.y, '  theta=', theta1
    
    odom_x=data.pose.pose.position.x
    odom_y=data.pose.pose.position.y
    odom_theta=theta1

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

    sub_pos_controller = rospy.Subscriber('elektron/mobile_base_controller/odom', Odometry, get_position_controller)
    sub_pos_gazebo = rospy.Subscriber('gazebo_odom', Odometry, get_position_gazebo)
    
    f1=open("tune_kwadrat.txt","wb")
    i=0
    while(i<=200):
	pom_odom_x=odom_x
	pom_odom_y=odom_y
	pom_odom_theta=odom_theta
	x=gazebo_odom_x
	y=gazebo_odom_y
	theta=gazebo_odom_theta
	diff_x=pom_odom_x-x
	diff_y=pom_odom_y-y
	diff_theta=pom_odom_theta-theta
		
	f1.write(str(pom_odom_x)+"    "+str(pom_odom_y)+"    "+str(pom_odom_theta)+"    "+str(x)+"    "+str(y)+"    "+str(theta)+"    "+str(diff_x)+"    "+str(diff_y)+"    "+str(diff_theta)+'\n')
	print 'CONTROLLER: x=',pom_odom_x, '  y=',pom_odom_y, '  theta=', pom_odom_theta
	print 'GAZEBOODOM: x=',x, '  y=',y, '  theta=', theta
	print 'BLAD      : x=',diff_x, '  y=',diff_y, '  theta=', diff_theta
	i=i+1
	rospy.sleep(0.2)

    f1.close()
    return "Koniec"


if __name__ == "__main__":

    rospy.init_node('Projekt1_Dane')
    collect_data()

    rospy.spin()