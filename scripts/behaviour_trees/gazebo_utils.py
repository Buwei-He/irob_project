#!/usr/bin/env python3

import numpy as np
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel, DeleteModelRequest, GetModelState
from geometry_msgs.msg import Pose

import signal
import sys


def ResetAruco():
	rospy.loginfo("Reset aruco cube...")
	aruco_cube_path = rospy.get_param("/gazebo_models_handler/aruco_cube_sdf")
	dlt_model_srv = "/gazebo/delete_model"
	spawn_model_srv = "/gazebo/spawn_sdf_model"

	# Wait for service providers
	rospy.wait_for_service(dlt_model_srv, timeout=30)
	rospy.wait_for_service(spawn_model_srv, timeout=30)
	delete_model_srv = rospy.ServiceProxy(dlt_model_srv, DeleteModel)

	# Initial pose of the cube
	initial_pose = Pose()
	initial_pose.position.x = -1.130530
	initial_pose.position.y = -6.653650
	initial_pose.position.z = 0.862500

	f = open(aruco_cube_path, 'r')
	sdffile = f.read()

	try:
		delete_cube = DeleteModelRequest('aruco_cube')
		delete_model_res = delete_model_srv(delete_cube)

	except rospy.ServiceException as e:
		print("Service call to gazebo delete_models failed: %s"%e)

	ros_rate = rospy.Rate(0.5)
	ros_rate.sleep()

	spawn_model_prox = rospy.ServiceProxy(spawn_model_srv, SpawnModel)
	spawn_model_prox("aruco_cube", sdffile, "/", initial_pose, "world")
      
	ros_rate.sleep()
	
	return 0


def GetModelPose(obj_name):
	try:
		model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		model_poses = model_state(obj_name, 'link')

	except rospy.ServiceException as e:
		rospy.loginfo("Get Model State service call failed:  {0}".format(e))
		
	return model_poses


if __name__ == "__main__":
    
    rospy.init_node('reset_aruco')
    try:
        ResetAruco()
        # print(GetModelPose("table_3_clone"))

    except rospy.ROSInterruptException:
        pass