#!/usr/bin/env python

import rospy
from geometry_msgs.msg import *
from stero_mobile_init.srv import *
import math
import time
from nav_msgs.msg import *
from tf.transformations import *


def projekt_1(x):

    velocity1=0.4	# predkosc katowa
    velocity2=0.2	# predkosc liniowa
    

    state = Twist()

    state.linear.x=0.0
    state.linear.y=0.0
    state.linear.z=0.0

    state.angular.x=0.0
    state.angular.y=0.0
    state.angular.z=0.0

    pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
    pub.publish(state)
    rospy.sleep(1)

    #"Podaj test: 1-ruch przod tyl, 2-obrot, 3-kwadrat "
    if x==1:

	time=8.42
	# rozpoczecie ruchu do przodu
	while(time>=0):
            state = Twist()
            state.linear.x=velocity2
            pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
            pub.publish(state)
	    time=time-0.01
	    rospy.sleep(0.01)

	# zakonczenie ruchu
        state = Twist()
        state.linear.x=0.0
        pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
        pub.publish(state)
	rospy.sleep(1.0)

	time=8.42
	# rozpoczecie ruchu w tyl
	while(time>=0):
            state = Twist()
            state.linear.x=-velocity2
            pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
            pub.publish(state)
	    time=time-0.01
	    rospy.sleep(0.01)

	# zakonczenie ruchu
        state = Twist()
        state.linear.x=0.0
        pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
        pub.publish(state)
	rospy.sleep(0.5)

    elif x==2:

	print "obrot"
	theta=2*math.pi
	state = Twist()

	# publikowanie parametrow obrotu
        pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
        pub.publish(state)

	time=10.8 #10.33
	while(time>=0):
            state = Twist()
	    state.angular.z=velocity1
            pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
            pub.publish(state)
	    time=time-0.01
	    rospy.sleep(0.01)

        # zakonczenie obracania
        state = Twist()
        state.angular.z=0.0
        pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
        pub.publish(state)
	rospy.sleep(0.5)

    elif x==3:

	print "kwadrat"
	for i in range (0, 4):

	    #obrot
	    time=2.53 #2.39
	    while(time>=0):
                state = Twist()
	        state.angular.z=velocity1
                pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
                pub.publish(state)
	        time=time-0.01
	        rospy.sleep(0.01)

            # zakonczenie obracania
            state = Twist()
            state.angular.z=0.0
            pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
            pub.publish(state)
	    rospy.sleep(0.5)


	    #prosta
            time=3.1
	    # rozpoczecie ruchu do przodu
	    while(time>=0):
                state = Twist()
                state.linear.x=velocity2
                pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
                pub.publish(state)
	        time=time-0.01
	        rospy.sleep(0.01)

	    # zakonczenie ruchu
            state = Twist()
            state.linear.x=0.0
            pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
            pub.publish(state)
	    rospy.sleep(2.0)

    return "Koniec trasy"



if __name__ == "__main__":

    rospy.init_node('PROJEKT1_2')

    if len(sys.argv) == 2:
        pom = float(sys.argv[1])
    else:
        sys.exit(1)

    projekt_1(pom)

    pub = rospy.Publisher('mux_vel_nav/cmd_vel', Twist, queue_size=1000)

    rospy.spin()
