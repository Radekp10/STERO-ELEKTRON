#!/usr/bin/env python

import sys
import rospy

from std_msgs.msg import *
from nav_msgs.msg import *
from stero_mobile_init.srv import *
from geometry_msgs.msg import *

import numpy as np


def lab1_2_client(positions):
    rospy.wait_for_service('lab1_2_control_srv')
    path = Path()
    for position in positions:
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y = position
        path.poses.append(pose)
    try:
        velocity_srv = rospy.ServiceProxy('lab1_2_control_srv', Lab1_2)
        resp1 = velocity_srv(path)
        return resp1.status
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e



if __name__ == "__main__":
    print sys.argv[1:]
    args = [arg for arg in sys.argv[1:]]
    positions = np.empty((len(args)/2, 2))
    if len(args) > 0 and len(args) % 2 == 0:
        positions[:] = np.array(args).reshape((len(args)/2, 2))
    else:
        print "Invalid arguments"
        sys.exit(1)
    
    print lab1_2_client(positions)    

