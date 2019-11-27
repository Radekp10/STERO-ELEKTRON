#!/usr/bin/env python

import rospy
from geometry_msgs.msg import *
from stero_mobile_init.srv import *
import math
import time
from nav_msgs.msg import *
from tf.transformations import *


def lab1_2_handle(req):
        
    #print req.path.poses
    velocity1=0.2	# predkosc katowa
    velocity2=0.2	# predkosc liniowa

    previous_x=0 # wspolrzedna poczatku odcinka
    previous_y=0	# wspolrzedna poczatku odcinka
    previous_theta=0	# poczatkowy kat przed pokonaniem odcinka

    new_x=0.0	# koniec odcinka wzgledem jego poczatku
    new_y=0.0	# koniec odcinka wzgledem jego poczatku
    new_theta=0.0	# kat o jaki trzeba sie obrocic, aby skierowac sie na koniec odcinka



    path=req.path.poses  

    for poseStamped in path:

        # KOLEJNY PUNKT
        pose = poseStamped.pose

	# wyliczenie wzglednych wspolrzednych konca odcinka
        new_x=pose.position.x-previous_x
        new_y=pose.position.y-previous_y

        distance=math.sqrt(math.pow(new_x,2) + math.pow(new_y,2))	# obliczenie przesuniecia
        theta=math.atan2(new_y, new_x)				# obliczenie kata obrotu
	print "THETA", theta

        state = Twist()

        state.linear.x=0.0
        state.linear.y=0.0
        state.linear.z=0.0

        state.angular.x=0.0
        state.angular.y=0.0

        # wybieranie kierunku obrotu
        new_theta=theta-previous_theta

	if new_theta>math.pi:
	    new_theta=new_theta-2*math.pi

	if new_theta<-math.pi:
	    new_theta=2*math.pi+new_theta

        if new_theta>=0.0 and new_theta<=math.pi:
            state.angular.z=velocity1
        else:
	    state.angular.z=-velocity1

        # publikowanie parametrow obrotu
        pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
        pub.publish(state)

        # czas obrotu
        time1=math.fabs(new_theta/state.angular.z)	
        rospy.sleep(time1)

        # zakonczenie obracania
        state = Twist()
        state.angular.z=0.0
        pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
        pub.publish(state)
	rospy.sleep(1)
	print "obrot koniec!!!!!!!!!!"

        # rozpoczecie ruchu w przod
        state = Twist()
        state.linear.x=velocity2
        pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
        pub.publish(state)
    
        # czas ruchu
        time2=math.fabs(distance/velocity2)
        rospy.sleep(time2)
	previous_sqrt=distance

        # zakonczenie ruchu
        state = Twist()
        state.linear.x=0.0
        pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
        pub.publish(state)
	rospy.sleep(1)	

	# przepisanie wspolrzednych nowego polozenia
        previous_x=pose.position.x
        previous_y=pose.position.y
        previous_theta=theta
	
	


    return "Koniec"

def lab1_2_server():
    rospy.init_node('lab1_2')
    s = rospy.Service('lab1_2_control_srv', Lab1_2, lab1_2_handle)
    
    pub = rospy.Publisher('mux_vel_nav/cmd_vel', Twist, queue_size=1000)

    rospy.spin()

if __name__ == "__main__":
    lab1_2_server()
