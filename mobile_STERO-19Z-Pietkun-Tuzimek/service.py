#!/usr/bin/env python

import rospy
from geometry_msgs.msg import *
from geometry_msgs.msg import Vector3
from stero_mobile_init.srv import *
import math
import time


global pub


def handle_velocity(req):
        
    #xpocz=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    velocity1=0.2	# predkosc katowa
    velocity2=0.2	# predkosc liniowa

    distance=math.sqrt(math.pow(req.x,2) + math.pow(req.y,2))	# obliczenie przesuniecia
    theta=math.atan2(req.y, req.x)				# obliczenie kata obrotu

    state = Twist()

    state.linear.x=0.0
    state.linear.y=0.0
    state.linear.z=0.0

    state.angular.x=0.0
    state.angular.y=0.0

    # wybieranie kierunku obrotu
    if theta>=0.0 and theta<=math.pi:
        state.angular.z=velocity1
    else:
	state.angular.z=-velocity1
    
    # print("atan2=",theta)
    # print("state.angular.z=",state.angular.z)

    # publikowanie parametrow obrotu
    pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
    pub.publish(state)

    # czas obrotu
    time1=theta/state.angular.z	
    rospy.sleep(time1)
    # print("time1=",time1)

    # zakonczenie obracania
    state = Twist()
    state.angular.z=0.0
    pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
    pub.publish(state)

    # rozpoczecie ruchu w przod
    state = Twist()
    state.linear.x=velocity2
    pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
    pub.publish(state)
    
    # czas ruchu
    time2=distance/velocity2
    rospy.sleep(time2)
    # print("time2=",time2)

    # zakonczenie ruchu
    state = Twist()
    state.linear.x=0.0
    pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
    pub.publish(state)

    return VelocityResponse("Koniec")

def velocity_server():
    rospy.init_node('velocity')
    s = rospy.Service('velocity_control_srv', Velocity, handle_velocity)
    
    pub = rospy.Publisher('mux_vel_nav/cmd_vel', Twist, queue_size=1000)

    rospy.spin()

if __name__ == "__main__":
    velocity_server()
