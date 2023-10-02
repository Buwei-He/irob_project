#!/usr/bin/env python3

import numpy as np
from numpy import linalg as LA

import rospy
from geometry_msgs.msg import Pose, Twist
from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState

from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

from gazebo_msgs.srv import SpawnModel, DeleteModel, DeleteModelRequest
from moveit_msgs.msg import MoveItErrorCodes

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

class StateMachine(object):
    """
    Custom state machine for mobile robot:
    State 0: Move the robot "manually" to door
    State 1: Tuck arm 
    State 2: Move the robot "manually" to chair
    State 3: Lower robot head service
    State 4: (Invalid)
    State -1: Error message
    """
    def __init__(self):
        
        self.node_name = "Student SM"
        self.aruco_pose_received = False
        self.robot_pose_received = False
        self.gripper_position = [1, 1]
        self.gripper_effort = [0, 0]
        self.prev_state = 0
        self.state = 0

        # Access rosparams
        # see launch_project.launch for details, and replace "placeholders" and blanks.

        # Move head service
        self.move_head_service_ns = rospy.get_param(rospy.get_name() + '/move_head_srv')
        # Pick and Place service
        self.pick_service_ns = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.place_service_ns = rospy.get_param(rospy.get_name() + '/place_srv')
        # Localization and Costmap service
        self.global_loc_service_ns = rospy.get_param(rospy.get_name() + '/global_loc_srv')
        self.clear_costmaps_service_ns = rospy.get_param(rospy.get_name() + '/clear_costmaps_srv')
        # Reset aruco service
        self.reset_aruco_service_ns = rospy.get_param(rospy.get_name() + '/reset_aruco_srv')

        # Pick/Place/Aruco pose topic
        self.pick_pose_topic_ns = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
        self.place_pose_topic_ns = rospy.get_param(rospy.get_name() + '/place_pose_topic')
        self.aruco_pose_topic_ns = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
        # Command velocity and Navigation goal topic
        self.cmd_vel_topic_ns = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.nav_goal_topic_ns = rospy.get_param(rospy.get_name() + '/nav_goal_topic')
        # Joint states topic
        self.joint_states_topic_ns = rospy.get_param(rospy.get_name() + '/joint_states_topic')

        print("\
            move_head_service: %s \n \
            pick_service: %s \n \
            place_service: %s \n \
            global_localize_service: %s \n \
            clear_costmaps_service: %s \n \
            pick_pose_topic: %s \n \
            place_pose_topic: %s \n \
            aruco_pose_topic: %s \n \
            cmd_vel_topic: %s \n \
            nal_goal_topic: %s \n \
            joint_states_topic: %s \n \
        " % (self.move_head_service_ns, self.pick_service_ns, self.place_service_ns,
            self.global_loc_service_ns, self.clear_costmaps_service_ns,
            self.pick_pose_topic_ns, self.place_pose_topic_ns, self.aruco_pose_topic_ns,
            self.cmd_vel_topic_ns, self.nav_goal_topic_ns, self.joint_states_topic_ns)
        )

        # Subscribe to topics
        self.robot_pose_subs = rospy.Subscriber(self.global_loc_service_ns, PoseWithCovarianceStamped, self.robot_pose_callback)
        self.aruco_pose_subs = rospy.Subscriber(self.aruco_pose_topic_ns, PoseStamped, self.aruco_pose_callback)
        self.joint_states_subs = rospy.Subscriber(self.joint_states_topic_ns, JointState, self.joint_states_callback)


        # Wait for service providers
        rospy.wait_for_service(self.move_head_service_ns, timeout=30)
        rospy.wait_for_service(self.pick_service_ns, timeout=30)
        rospy.wait_for_service(self.place_service_ns, timeout=30)
        rospy.wait_for_service(self.reset_aruco_service_ns, timeout=30)

        # Instantiate publishers
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic_ns, Twist, queue_size=10)

        # Setup action clients 
        # play_motion: robot arm action
        rospy.loginfo("%s: Waiting for play_motion action server...", self.node_name)
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)
        if not self.play_motion_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /play_motion action server", self.node_name)
            exit()
        rospy.loginfo("%s: Connected to play_motion action server", self.node_name)
        # move_base: robot base action
        rospy.loginfo("%s: Waiting for move_base action server...", self.node_name)
        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)
        if not self.move_base_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("%s: Could not connect to /move_base action server", self.node_name)
            exit()
        rospy.loginfo("%s: Connected to move_base action server", self.node_name)

        # Init state machine
        rospy.sleep(3)
        self.check_states()


    # callback

    def aruco_pose_callback(self, aruco_pose_msg):
        """ Update aruco pose. """
        valid = aruco_pose_msg is not None
        if valid:
            self.aruco_pose_received = True
        return valid


    def robot_pose_callback(self, robot_pose_msg):
        """ Update robot pose. """
        valid = robot_pose_msg is not None
        if valid:
            self.robot_pose_received = True
        return valid


    def joint_states_callback(self, joint_states_msg):
        """ Update gripper joints state. """
        valid = joint_states_msg is not None
        if valid:
            self.gripper_position = [joint_states_msg.position[7], joint_states_msg.position[8]]
            self.gripper_effort = [joint_states_msg.effort[7], joint_states_msg.effort[8]]
            if self.state == 6:
                self.aruco_fall_detect()
        return valid


    def localization(self):
        # class type unknown
        try:
            localization_service = rospy.ServiceProxy(self.global_loc_service_ns, Empty)
            localization_req = localization_service()
            # success = True

            return localization_req.success
            # return success
        
        except rospy.ServiceException as e:
            rospy.loginfo("Service call to localization server failed: %s" % e)
            return False


    def navigate_to_pose(self, target_pose):
        try:
            rospy.loginfo("%s: Navigating to the target pose...", self.node_name)
            goal = MoveBaseGoal()
            goal.target_pose = target_pose
            self.move_base_ac.send_goal(goal)
            self.move_base_ac.wait_for_result(rospy.Duration(100.0))
            success = self.move_base_ac.get_state()
        
        except rospy.ROSException as e:
            rospy.loginfo("Navigation error: %s", e)
            success = False

        if success:
            rospy.loginfo("%s: Target pose reached.", self.node_name)
        else:
            rospy.logerr("%s: Failed to reach the target, please retry or handle it.", self.node_name)
            self.move_base_ac.cancel_goal()

        return success
    

    def move_base_motion(self, move_msg, time):
        rate_per_sec = 10
        rate = rospy.Rate(rate_per_sec)
        cnt = 0
        rospy.loginfo("%s: Moving the base...", self.node_name)

        while not rospy.is_shutdown() and cnt < int(time * rate_per_sec):
            self.cmd_vel_pub.publish(move_msg)
            rate.sleep()
            cnt += 1
        
        success = (cnt > int(time * rate_per_sec) - 2)
        if success:
            rospy.loginfo("%s: Arm tuck: ", self.play_motion_ac.get_result())
        else:
            rospy.logerr("%s: Tuck arm failed, reset simulation.", self.node_name)
            self.play_motion_ac.cancel_goal()

        return success


    def update_costmap(self):
        try:
            clear_costmap_service = rospy.ServiceProxy(self.clear_costmaps_service_ns, Empty)
            clear_costmap_req = clear_costmap_service()
            success = True

            # return clear_costmap_req.success
            return success
        
        except rospy.ServiceException as e:
            rospy.loginfo("Service call to clear_costmap server failed: %s" % e)
            return False


    def tuck_arm_motion(self):
        rospy.loginfo("%s: Tucking the arm...", self.node_name)
        goal = PlayMotionGoal()
        goal.motion_name = 'home'
        goal.skip_planning = True
        self.play_motion_ac.send_goal(goal)
        success = self.play_motion_ac.wait_for_result(rospy.Duration(100.0))

        if success:
            rospy.loginfo("%s: Tuck arm succeeded!", self.play_motion_ac.get_result())
            
        else:
            rospy.loginfo("%s: Tuck arm failed!", self.node_name)
            self.play_motion_ac.cancel_goal()

        return success
    

    def aruco_fall_detect(self):
        # gripper distance and effort (force)
        disp_threshold = 0.02
        force_threshold = 0.5
        p_left, p_right = self.gripper_position[0], self.gripper_position[1]
        f_left, f_right = self.gripper_effort[0], self.gripper_effort[1]
        closed = (p_left < disp_threshold) and (p_right < disp_threshold)
        force_reduced = (abs(f_left) < force_threshold) and (abs(f_right) < force_threshold)
        if closed and force_reduced:
            rospy.loginfo("%s: Aruco cube is thrown away!", self.node_name)
            rospy.loginfo("%s: Position: %0.2f, %0.2f", self.node_name, p_left, p_right)
            rospy.loginfo("%s: Effort: %0.2f, %0.2f", self.node_name, f_left, f_right)
        # visual detect?
            return False
        
        else: return True
    

    def move_head_motion(self, direction="down"):
        try:
            rospy.loginfo("%s: Moving robot head", self.node_name)
            move_head_srv = rospy.ServiceProxy(self.move_head_service_ns, MoveHead)
            move_head_req = move_head_srv(direction)
            success = move_head_req.success

            if success:
                rospy.loginfo("%s: Move head down succeeded!", self.node_name)
            else:
                rospy.loginfo("%s: Move head down failed!", self.node_name)
                
            return success

        except rospy.ServiceException as e:
            rospy.loginfo("Service call to move_head server failed: %s" % e)
            return False
    

    def reset_aruco(self):

        rospy.loginfo("%s: Reset aruco cube...", self.node_name)
        aruco_cube_path = rospy.get_param("/gazebo_models_handler/aruco_cube_sdf")
        self.dlt_model_srv = "/gazebo/delete_model"
        self.spawn_model_srv = "/gazebo/spawn_sdf_model"

        # Wait for service providers
        rospy.wait_for_service(self.dlt_model_srv, timeout=30)
        rospy.wait_for_service(self.spawn_model_srv, timeout=30)
        self.delete_model_srv = rospy.ServiceProxy(self.dlt_model_srv, DeleteModel)

        rospy.loginfo("%s: Services received!", self.node_name)

        # Initial pose of the cube
        initial_pose = Pose()
        initial_pose.position.x = -1.130530
        initial_pose.position.y = -6.653650
        initial_pose.position.z = 0.862500

        f = open(aruco_cube_path,'r')
        sdffile = f.read()

        rospy.loginfo("%s: Calling gazebo delete_models", self.node_name)
        try:
            delete_cube = DeleteModelRequest('aruco_cube')
            delete_model_res = self.delete_model_srv(delete_cube)

        except rospy.ServiceException as e:
            print("Service call to gazebo delete_models failed: %s"%e)

        rospy.loginfo("%s: Respawn aruco model...", self.node_name)
        spawn_model_prox = rospy.ServiceProxy(self.spawn_model_srv, SpawnModel)
        spawn_model_prox("aruco_cube", sdffile, "/", initial_pose, "world")
        
        return 0


    def check_states(self):

        while not rospy.is_shutdown():

            # State 0:  Tuck arm 
            if self.state == 0:
                self.prev_state = self.state

                move_msg = Twist()
                move_msg.linear.x = -0.5
                self.move_base_motion(move_msg, time=1)

                # tuuck arm
                success = self.tuck_arm_motion()
                
                # next state
                self.state = 1 if success else -1
                rospy.sleep(1)

            # State 1: Localize
            if self.state == 1:
                self.prev_state = self.state

                move_msg = Twist()
                move_msg.angular.z = 1
                success = self.move_base_motion(move_msg, time=5.8)

                # update costmap
                self.update_costmap()

                # next state
                self.state = 2 if success else -1
                rospy.sleep(1)

            # State 2:  Navigate to pick pose
            if self.state == 2:
                self.prev_state = self.state
                target_pose = rospy.wait_for_message(self.pick_pose_topic_ns, PoseStamped, 5)
                success = self.navigate_to_pose(target_pose)
                
                # next state
                self.state = 3 if success else -1
                rospy.sleep(1)

            # State 3:  Lower robot head service
            if self.state == 3:
                try:
                    rospy.loginfo("%s: Lowering robot head", self.node_name)
                    move_head_srv = rospy.ServiceProxy(self.move_head_service_ns, MoveHead)
                    move_head_req = move_head_srv("down")
                    self.prev_state = self.state
                    if move_head_req.success:
                        self.state = 4
                        rospy.loginfo("%s: Move head down succeded!", self.node_name)
                    else:
                        self.state = -1
                        rospy.loginfo("%s: Move head down failed!", self.node_name)
                        
                    rospy.sleep(3)

                except rospy.ServiceException as e:
                    print("Service call to move_head server failed: %s"%e)
                    

            # State 4:  Detect aruco cube & update cost map
            if self.state == 4:

                move_msg = Twist()
                move_msg.angular.z = 0.25

                rate = rospy.Rate(10)
                cnt = 0
                rospy.loginfo("%s: Checking if cube is in sight...", self.node_name)

                # re-initialize and detect aruco pose
                self.aruco_pose_received = False
                while not rospy.is_shutdown() and not self.aruco_pose_received and cnt < 300:
                    self.cmd_vel_pub.publish(move_msg)
                    cnt += 1
                    rate.sleep()
                rospy.loginfo(str(cnt))

                self.previous_state = self.state
                self.state = 5
                rospy.sleep(3)


            # State 5:  Pick up service
            if self.state == 5:
                try:
                    rospy.loginfo("%s: Pick up object", self.node_name)
                    pick_service = rospy.ServiceProxy(self.pick_service_ns, SetBool)
                    pick_req = pick_service()
                    self.prev_state = self.state
                    success = self.aruco_fall_detect()

                    if pick_req.success and success:
                        self.state = 6
                        rospy.loginfo("%s: Pick up succeeded!", self.node_name)
                    else:
                        self.state = 2
                        rospy.loginfo("%s: Pick up failed!", self.node_name)
                        self.reset_aruco()

                    rospy.sleep(3)

                except rospy.ServiceException as e:
                    print("Service call to move_head server failed: %s"%e)


            # State 6:  Navigate to pick pose
            if self.state == 6:
                self.prev_state = self.state
                target_pose = rospy.wait_for_message(self.place_pose_topic_ns, PoseStamped, 5)
                success = self.navigate_to_pose(target_pose)
                
                # next state
                self.state = 7 if success else -1
                rospy.sleep(1)


            # State 7:  Place service
            if self.state == 7:
                try:
                    self.update_costmap()

                    rospy.loginfo("%s: Place object", self.node_name)
                    place_service = rospy.ServiceProxy(self.place_service_ns, SetBool)
                    place_req = place_service()
                    self.prev_state = self.state

                    if place_req.success:
                        self.state = 8
                        rospy.loginfo("%s: Place succeeded!", self.node_name)
                    else:
                        self.state = 7
                        rospy.loginfo("%s: Place failed!", self.node_name)

                    rospy.sleep(3)

                except rospy.ServiceException as e:
                    print("Service call to move_head server failed: %s"%e)

            # Error handling
            if self.state == -1:
                rospy.logerr("%s: State machine failed. Check your code and try again!", self.node_name)
                return

        rospy.loginfo("%s: State machine finished!", self.node_name)
        return

	

if __name__ == "__main__":

	rospy.init_node('main_state_machine')
	try:
		StateMachine()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
