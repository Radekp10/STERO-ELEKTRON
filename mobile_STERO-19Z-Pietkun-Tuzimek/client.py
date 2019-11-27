#!/usr/bin/env python

import sys
import rospy

from std_msgs.msg import String
from stero_mobile_init.srv import *


def velocity_client(x,y,z,roll,pitch,yaw):
    rospy.wait_for_service('velocity_control_srv')
    try:
        velocity_srv = rospy.ServiceProxy('velocity_control_srv', Velocity)
        resp1 = velocity_srv(x,y,z,roll,pitch,yaw)
        return resp1.status
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y z roll pitch yaw]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 7:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
        roll = float(sys.argv[4])
        pitch = float(sys.argv[5])
        yaw = float(sys.argv[6])
    else:
        print usage()
        sys.exit(1)
    print "Requesting params: %s %s %s %s %s %s"%(x,y,z,roll,pitch,yaw)
    print "%s, %s, %s, %s, %s, %s -> %s"%(x,y,z,roll,pitch,yaw, velocity_client(x,y,z,roll,pitch,yaw))
