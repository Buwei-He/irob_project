#!/usr/bin/env python3

import numpy as np
from numpy import linalg as LA

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState

from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

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
        self.aruco_pose = None
        self.robot_pose = None
        self.prev_state = 2
        self.state = 2

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
        
        # Pick/Place/Aruco pose topic
        self.pick_pose_topic_ns = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
        self.place_pose_topic_ns = rospy.get_param(rospy.get_name() + '/place_pose_topic')
        self.aruco_pose_topic_ns = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
        # Command velocity and Navigation goal topic
        self.cmd_vel_topic_ns = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.nav_goal_topic_ns = rospy.get_param(rospy.get_name() + '/nav_goal_topic')

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
        " % (self.move_head_service_ns, self.pick_service_ns, self.place_service_ns,
            self.global_loc_service_ns, self.clear_costmaps_service_ns,
            self.pick_pose_topic_ns, self.place_pose_topic_ns, self.aruco_pose_topic_ns,
            self.cmd_vel_topic_ns, self.nav_goal_topic_ns)
        )

        # Subscribe to topics
        self.robot_pose_subs = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.robot_pose_callback)
        self.aruco_pose_subs = rospy.Subscriber(self.aruco_pose_topic_ns, PoseStamped, self.aruco_pose_callback)

        # Wait for service providers
        rospy.wait_for_service(self.move_head_service_ns, timeout=30)
        rospy.wait_for_service(self.pick_service_ns, timeout=30)
        rospy.wait_for_service(self.place_service_ns, timeout=30)

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


    def aruco_pose_callback(self, aruco_pose_msg):
        """ Update aruco pose. """
        valid = aruco_pose_msg is not None
        if valid:
            self.aruco_pose = aruco_pose_msg
        return valid


    def robot_pose_callback(self, robot_pose_msg):
        """ Update robot pose. """
        valid = robot_pose_msg is not None
        if valid:
            self.robot_pose = robot_pose_msg
        return valid


    def localization(self):
        # class type unknown
        localization_service = rospy.ServiceProxy(self.global_loc_service_ns, Empty)
        localization_status = localization_service()

        return 0


    def navigate_to_pose(self, target_pose):
        rospy.loginfo("%s: Navigating to the target pose...", self.node_name)
        goal = MoveBaseGoal()
        goal.target_pose = target_pose
        self.move_base_ac.send_goal(goal)
        success = self.move_base_ac.wait_for_result(rospy.Duration(100.0))

        if success:
            rospy.loginfo("%s: Target pose reached.", self.node_name)
        else:
            self.move_base_ac.cancel_goal()
            rospy.logerr("%s: Failed to reach the target, please retry or handle it.", self.node_name)
        return success
    

    def check_states(self):

        while not rospy.is_shutdown():
            
            # State 0: Localize
            if self.state == 0:
                move_msg = Twist()
                move_msg.linear.x = 1

                rate = rospy.Rate(10)
                converged = False
                cnt = 0
                rospy.loginfo("%s: Moving towards door", self.node_name)
                while not rospy.is_shutdown() and cnt < 5:
                    self.cmd_vel_pub.publish(move_msg)
                    rate.sleep()
                    cnt = cnt + 1

                # next state
                self.state = 1
                rospy.sleep(1)

            # State 1:  Tuck arm 
            if self.state == 1:
                rospy.loginfo("%s: Tucking the arm...", self.node_name)
                goal = PlayMotionGoal()
                goal.motion_name = 'home'
                goal.skip_planning = True
                self.play_motion_ac.send_goal(goal)
                success = self.play_motion_ac.wait_for_result(rospy.Duration(100.0))
                self.prev_state = self.state

                if success:
                    self.state = 2
                    rospy.loginfo("%s: Arm tuck: ", self.play_motion_ac.get_result())
                    
                else:
                    self.play_motion_ac.cancel_goal()
                    self.state = -1
                    rospy.logerr("%s: Tuck arm failed, reset simulation.", self.node_name)

                rospy.sleep(1)

            # State 2:  Navigate
            if self.state == 2:
                rospy.loginfo("%s: Navigating to cube...", self.node_name)
                target_pose = rospy.wait_for_message(self.pick_pose_topic_ns, PoseStamped, 5)
                success = self.navigate_to_pose(target_pose)
                self.prev_state = self.state
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

            # State 4:  Detect aruco cube
            if self.state == 4:

                move_msg = Twist()
                move_msg.angular.z = 0.8

                rate = rospy.Rate(10)
                cnt = 0
                self.aruco_pose_rcv = False
                rospy.loginfo("%s: Checking if cube is in sight...", self.node_name)
                while not rospy.is_shutdown() and self.aruco_pose_rcv == False and cnt < 100:
                    self.cmd_vel_pub.publish(move_msg)
                    cnt += 1
                    rate.sleep()

                self.previous_state = self.state
                self.state = 4
                rospy.sleep(3)


            # State 5:  Pick up service
            if self.state == 5:
                try:
                    rospy.loginfo("%s: Pick up object", self.node_name)
                    pick_service = rospy.ServiceProxy(self.pick_service_ns, SetBool)
                    pick_req = pick_service()
                    self.prev_state = self.state

                    if pick_req.success:
                        self.state = 6
                        rospy.loginfo("%s: Pick up succeded!", self.node_name)
                    else:
                        self.state = -1
                        rospy.loginfo("%s: Pick up failed!", self.node_name)

                    rospy.sleep(3)

                except rospy.ServiceException as e:
                    print("Service call to move_head server failed: %s"%e)

            # State 6:  Place service
            if self.state == 6:
                try:
                    rospy.loginfo("%s: Place object", self.node_name)
                    place_service = rospy.ServiceProxy(self.place_service_ns, SetBool)
                    place_req = place_service()
                    self.prev_state = self.state

                    if place_req.success:
                        self.state = 7
                        rospy.loginfo("%s: Place succeded!", self.node_name)
                    else:
                        self.state = -1
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
