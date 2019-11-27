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
    velocity2=0.5	# predkosc liniowa
    

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
	print "ruch"
	distance=5.0	# dystans
	time2=math.fabs(distance/velocity2)
	state = Twist()

        # rozpoczecie ruchu do przodu
        
        print "ruch4"
        # czas ruchu


	# rozpoczecie ruchu do przodu
        state = Twist()
        state.linear.x=velocity2
        pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
        pub.publish(state)
        print "ruch4"
        # czas ruchu
        rospy.sleep(time2)
        print "ruch5"
        # zakonczenie ruchu
        state = Twist()
        state.linear.x=0.0
        pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
        pub.publish(state)
	rospy.sleep(1.0)
        print "ruch6"

	# rozpoczecie ruchu w tyl
        state = Twist()
        state.linear.x=-velocity2
        pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
        pub.publish(state)
        print "ruch4"
        # czas ruchu
        rospy.sleep(time2)
        print "ruch5"
        # zakonczenie ruchu
        state = Twist()
        state.linear.x=0.0
        pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
        pub.publish(state)
	rospy.sleep(0.5)
        print "ruch6"

    elif x==2:

	print "obrot"
	theta=2*math.pi
	state = Twist()
        state.angular.z=velocity1

	# publikowanie parametrow obrotu
        pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
        pub.publish(state)

        # czas obrotu
        time1=math.fabs(theta/state.angular.z)	
        rospy.sleep(time1)

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
	    theta=math.pi/2
            state = Twist()
            state.angular.z=velocity1

	    # publikowanie parametrow obrotu
            pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
            pub.publish(state)

            # czas obrotu
            time1=math.fabs(theta/state.angular.z)	
            rospy.sleep(time1)

            # zakonczenie obracania
            state = Twist()
            state.angular.z=0.0
            pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
            pub.publish(state)
	    rospy.sleep(0.5)


	    #prosta

	    distance=2.0	# dystans

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
	    rospy.sleep(0.5)

    return "Koniec trasy"



if __name__ == "__main__":

    rospy.init_node('PROJEKT1')

    if len(sys.argv) == 2:
        pom = float(sys.argv[1])
    else:
        sys.exit(1)

    projekt_1(pom)

    pub = rospy.Publisher('mux_vel_nav/cmd_vel', Twist, queue_size=1000)

    rospy.spin()

