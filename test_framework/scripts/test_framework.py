#!/usr/bin/python

from std_srvs.srv import Empty

import rospy

from collections import namedtuple

import numpy as np
import math
import random
import time

import shlex, subprocess
import threading

class TrajectoryFollowingConfiguration(object):
    def __init__(self):
        pass


class MpcFollowerConfiguration(TrajectoryFollowingConfiguration):
    def __init__(self):
        pass

class TestFramework(object):

    def reset_simulation(self):
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
            reset_world()
            rospy.loginfo("Successful reset of simulation")
        except:
            rospy.logerr("Unable to reset simulation")
    
    def start_mpc_converter(self):
        t1 = subprocess.call(["rosrun", "waypoint_follower", "mpc_waypoints_converter"])

    def start_mpc_follower(self)
        t0 = subprocess.call(["rosrun", "waypoint_follower", "mpc_follower"])
        
    def run(self):
        #weight_heading_error = random.uniform(0.0, 10.0)
        weight_heading_error = 1.4
        #weight_steering_error = random.uniform(0.0, 1.0)
        weight_steering_error = 0.3
        weight_lat_error = 10
        rospy.loginfo("Chosen values: {0} {1}".format(weight_heading_error, weight_steering_error))
        rospy.set_param('/mpc_follower/mpc_weight_heading_error', weight_heading_error)
        rospy.set_param('/mpc_follower/mpc_weight_lat_error', weight_lat_error)
        rospy.set_param('/mpc_follower/mpc_weight_steering_input', weight_steering_error)
        
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        
        
            

    


def main():
    rospy.init_node("test_framework_jkk")
    rospy.loginfo("Starting test monitor")
    testframework = TestFramework()
    testframework.reset_simulation()
    testframework.run()

if __name__=="__main__":
    main()

    