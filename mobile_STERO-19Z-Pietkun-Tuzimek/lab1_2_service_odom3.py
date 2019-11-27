#!/usr/bin/env python

import rospy
from geometry_msgs.msg import *
from stero_mobile_init.srv import *
import math
import time
from nav_msgs.msg import *
from tf.transformations import *


global pub

odom_x=0.0
odom_y=0.0
odom_theta=0.0


def get_position(data):

    global odom_x 
    global odom_y
    global odom_theta
    orientation=[data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
    _, _, theta1 = euler_from_quaternion(orientation)
    #print 'x=',data.pose.pose.position.x, '  y=', data.pose.pose.position.y, '  theta=', theta1
    
    odom_x=data.pose.pose.position.x
    odom_y=data.pose.pose.position.y
    odom_theta=theta1


def lab1_2_handle(req):
        
    #print req.path.poses
    velocity1=0.2	# predkosc katowa
    velocity2=0.2	# predkosc liniowa

    sub_pos = rospy.Subscriber('elektron/mobile_base_controller/odom', Odometry, get_position)

    previous_x=odom_x # wspolrzedna poczatku odcinka
    previous_y=odom_y	# wspolrzedna poczatku odcinka
    previous_theta=odom_theta	# poczatkowy kat przed pokonaniem odcinka

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

        #distance=math.sqrt(math.pow(new_x,2) + math.pow(new_y,2))	# obliczenie przesuniecia
        theta=math.atan2(new_y, new_x)				# obliczenie kata obrotu
	print "Nowy punkt do osiagniecia: x=", pose.position.x, "y=", pose.position.y, "theta=", theta

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
        #time1=math.fabs(new_theta/state.angular.z)	
        #rospy.sleep(time1)


	# obracenie sie do momentu, gdy odczyty z odometrii wskaza, ze osiagnieto zadane ustawienie
	if math.fabs(odom_theta-theta)<math.pi:

	    previous_difference=math.fabs(odom_theta-theta)
	    new_difference=previous_difference

	    while new_difference<=previous_difference or math.fabs(new_difference-previous_difference)<0.000001:
	        previous_difference=new_difference
 	        new_difference=math.fabs(odom_theta-theta)
	        rospy.sleep(0.01)

	else:	#roznica miedzy odom_theta i theta jest wieksza od 180 stopni, czyli podczas obrotu konieczne bedzie przejscie
		#przez punkt pi/-pi, poczatkowo roznica miedzy odom_theta i theta bedzie rosnac,
		#dopiero po przejsciu przez punkt pi/-pi roznica miedzy odom_theta i theta zacznie malec, wiec dopiero wtedy bedzie
		#sens ja sprawdzac

	    if odom_theta<0 and theta>0:
		print "Oczekiwanie na przejscie przez punkt pi/-pi, obrot w prawo"
                while odom_theta<0.0:		#czekamy az robot przekroczy polozenie pi/-pi, robot obraca sie w prawo
		    pass	#pusta instrukcja w pythonie


	    if odom_theta>0 and theta<0:	#czekamy az robot przekroczy polozenie pi/-pi, robot obraca sie w lewo
		print "Oczekiwanie na przejscie przez punkt pi/-pi, obrot w lewo"
		while odom_theta>0.0:
    		    pass	#pusta instrukcja w pythonie


	    previous_difference=math.fabs(odom_theta-theta)
            new_difference=previous_difference

	    while new_difference<=previous_difference or math.fabs(new_difference-previous_difference)<0.000001:
	        previous_difference=new_difference
 	        new_difference=math.fabs(odom_theta-theta)
		rospy.sleep(0.01)


        # zakonczenie obracania
        state = Twist()
        state.angular.z=0.0
        pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
        pub.publish(state)
	print "Koniec obrotu"
	rospy.sleep(1)


        # rozpoczecie ruchu w przod
        state = Twist()
        state.linear.x=velocity2
        pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
        pub.publish(state)
    
        # czas ruchu
        #time2=math.fabs(distance/velocity2)
        #rospy.sleep(time2)
	#previous_sqrt=distance
	#i=0


	# ruch do przodu do momentu, gdy odczyty z odometrii wskaza, ze zaczynamy sie oddalac od zadanego punktu
	previous_distance=math.sqrt(math.pow(pose.position.x-odom_x,2)+math.pow(pose.position.y-odom_y,2))
	new_distance=previous_distance

	while new_distance<=previous_distance or math.fabs(new_distance-previous_distance)<0.000001:
	    #drugi warunek to zabezpieczenie przed szumami pomiarowymi
	    previous_distance=new_distance
	    new_distance=math.sqrt(math.pow(pose.position.x-odom_x,2)+math.pow(pose.position.y-odom_y,2))
	    rospy.sleep(0.01)

        """
	while math.sqrt(math.pow(pose.position.x-odom_x,2)+math.pow(pose.position.y-odom_y,2))<=previous_sqrt:# and i<4:
	    previous_sqrt=math.sqrt(math.pow(pose.position.x-odom_x,2)+math.pow(pose.position.y-odom_y,2))
	    #if math.sqrt(math.pow(pose.position.x-odom_x,2)+math.pow(pose.position.y-odom_y,2))>previous_sqrt:
		#i=i+i
            rospy.sleep(0.01)
	"""

        # zakonczenie ruchu
        state = Twist()
        state.linear.x=0.0
        pub=rospy.Publisher('mux_vel_nav/cmd_vel',Twist,queue_size=100)
        pub.publish(state)
        print "Koniec ruchu"
	print "Pozycja z odometrii: x=", odom_x, " y=", odom_y, " theta=", odom_theta
	print ""
	rospy.sleep(1)	

	# przepisanie wspolrzednych nowego polozenia
        previous_x=pose.position.x
        previous_y=pose.position.y
        previous_theta=theta
	

    return "Koniec trasy"



def lab1_2_server():
    rospy.init_node('lab1_2')
    s = rospy.Service('lab1_2_control_srv', Lab1_2, lab1_2_handle)
    
    pub = rospy.Publisher('mux_vel_nav/cmd_vel', Twist, queue_size=1000)

    rospy.spin()



if __name__ == "__main__":
    lab1_2_server()


