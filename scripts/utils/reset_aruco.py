#!/usr/bin/env python3

import numpy as np
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel, DeleteModelRequest 
from geometry_msgs.msg import Pose

import signal
import sys

def reset_aruco():
	rospy.loginfo("Reset aruco cube...")
	aruco_cube_path = rospy.get_param("/gazebo_models_handler/aruco_cube_sdf")
	dlt_model_srv = "/gazebo/delete_model"
	spawn_model_srv = "/gazebo/spawn_sdf_model"

	# Wait for service providers
	rospy.wait_for_service(dlt_model_srv, timeout=30)
	rospy.wait_for_service(spawn_model_srv, timeout=30)
	delete_model_srv = rospy.ServiceProxy(dlt_model_srv, DeleteModel)

	rospy.loginfo("Services received!")

	# Initial pose of the cube
	initial_pose = Pose()
	initial_pose.position.x = -1.130530
	initial_pose.position.y = -6.653650
	initial_pose.position.z = 0.862500

	f = open(aruco_cube_path, 'r')
	sdffile = f.read()
	rospy.loginfo("Path: %s", aruco_cube_path)

	rospy.loginfo("Calling gazebo delete_models")
	try:
		delete_cube = DeleteModelRequest('aruco_cube')
		delete_model_res = delete_model_srv(delete_cube)

	except rospy.ServiceException as e:
		print("Service call to gazebo delete_models failed: %s"%e)

	rate = rospy.Rate(0.4).sleep()

	rospy.loginfo("Respawn aruco model...")
	spawn_model_prox = rospy.ServiceProxy(spawn_model_srv, SpawnModel)
	spawn_model_prox("aruco_cube", sdffile, "/", initial_pose, "world")
	
	return 0

if __name__ == "__main__":
    
    rospy.init_node('reset_aruco')
    try:
        reset_aruco()
        # reset_aruco()

    except rospy.ROSInterruptException:
        pass