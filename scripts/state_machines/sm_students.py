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

from actionlib import SimpleActionClient, SimpleGoalState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

from gazebo_utils import GetModelPose, ResetAruco
from moveit_msgs.msg import MoveItErrorCodes

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

# Read table pose from gazebo
is_reading_table_pose = True

class StateMachine(object):
    """
    Custom state machine for mobile robot:
    State -1: Error message
    State 0: Move the robot "manually" to door
    State 1: Tuck arm 
    State 2: Move the robot "manually" to chair
    State 3: Lower robot head service
    State 4: (Invalid)
    
    """
    def __init__(self):
        
        self.node_name = "Student SM"
        self.aruco_pose_received = False
        self.prev_state = 1
        self.state = 1

        # Access rosparams
        # see launch_project.launch for details, and replace "placeholders" and blanks.

        # Move head service
        self.move_head_service_ns = rospy.get_param(rospy.get_name() + '/move_head_srv')
        # Pick and Place service
        self.pick_service_ns = rospy.get_param(rospy.get_name() + '/pick_srv')
        self.place_service_ns = rospy.get_param(rospy.get_name() + '/place_srv')

        # Pick/Place/Aruco pose topic
        self.pick_pose_topic_ns = rospy.get_param(rospy.get_name() + '/pick_pose_topic')
        self.place_pose_topic_ns = rospy.get_param(rospy.get_name() + '/place_pose_topic')
        self.aruco_pose_topic_ns = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')

        # # hard-code cube pose; send it to aruco_pose_topic if not using visual detection
        # self.cube_pose = rospy.get_param(rospy.get_name() + '/cube_pose')

        # Command velocity topic
        self.cmd_vel_topic_ns = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')

        # Subscribe to topics
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


    # callback

    def aruco_pose_callback(self, aruco_pose_msg):
        """ Update aruco pose. """
        valid = aruco_pose_msg is not None
        if valid:
            self.aruco_pose_received = True
        return valid


    def navigate(self, target_pose):
        try:
            rospy.loginfo("%s: Navigating to the target pose...", self.node_name)
            goal = MoveBaseGoal()
            goal.target_pose = target_pose
            self.move_base_ac.send_goal(goal)
            self.move_base_ac.wait_for_result(rospy.Duration(100.0))
            success = self.move_base_ac.get_result()
        
        except rospy.ROSException as e:
            rospy.loginfo("Navigation error: %s", e)
            success = False

        if success:
            rospy.loginfo("%s: Target pose reached.", self.node_name)
        else:
            rospy.logerr("%s: Failed to reach the target, please retry or handle it.", self.node_name)
            self.move_base_ac.cancel_goal()

        return success
    

    def go(self, move_msg, time):
        rate_per_sec = 10
        rate = rospy.Rate(rate_per_sec)
        cnt = 0
        rospy.loginfo("%s: Moving the base...", self.node_name)

        while not rospy.is_shutdown() and cnt < int(time * rate_per_sec):
            self.cmd_vel_pub.publish(move_msg)
            rate.sleep()
            cnt += 1
        
        success = True

        return success


    def tuck_arm(self):
        rospy.loginfo("%s: Tucking the arm...", self.node_name)
        goal = PlayMotionGoal()
        goal.motion_name = 'home'
        goal.skip_planning = True
        self.play_motion_ac.send_goal(goal)
        self.play_motion_ac.wait_for_result(rospy.Duration(100.0))
        success = self.play_motion_ac.get_result()
        # rospy.loginfo(self.play_motion_ac.get_result().error_code)
        if success or success.error_code == 1:
            rospy.loginfo("%s: Tuck arm succeeded!", self.node_name)
            
        else:
            rospy.loginfo("%s: Tuck arm failed!", self.node_name)
            self.play_motion_ac.cancel_goal()

        return success
    

    def move_head(self, direction="down"):
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
    

    def check_states(self):

        while not rospy.is_shutdown():
            # State -1: Error handling
            if self.state == -1:
                rospy.logerr("%s: State machine failed. Check your code and try again!", self.node_name)
                return

            # State 0:  Exit
            if self.state == 0:
                ResetAruco()
                rospy.loginfo("%s: Exit!", self.node_name)
                exit()

            # State 1:  Tuck arm 
            if self.state == 1:
                self.prev_state = self.state

                ResetAruco()

                move_msg = Twist()
                move_msg.linear.x = -0.5
                self.go(move_msg, time=1)

                # tuck arm
                success = self.tuck_arm()
                
                # next state
                self.state = 2 if success else -1
                rospy.sleep(1)

            # State 2:  Navigate to pick pose
            if self.state == 2:
                self.prev_state = self.state
                target_pose = rospy.wait_for_message(self.pick_pose_topic_ns, PoseStamped, 5)
                success = self.navigate(target_pose)
                
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
                        rospy.loginfo("%s: Move head down succeeded!", self.node_name)
                    else:
                        self.state = -1
                        rospy.loginfo("%s: Move head down failed!", self.node_name)
                        
                    rospy.sleep(1)

                except rospy.ServiceException as e:
                    print("Service call to move_head server failed: %s"%e)
                    

            # State 4:  Detect aruco cube
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

                self.previous_state = self.state
                self.state = 5
                rospy.sleep(1)


            # State 5:  Pick up service
            if self.state == 5:
                try:
                    rospy.loginfo("%s: Pick up object", self.node_name)
                    pick_service = rospy.ServiceProxy(self.pick_service_ns, SetBool)
                    pick_req = pick_service()
                    self.prev_state = self.state

                    if pick_req.success:
                        self.state = 6
                        rospy.loginfo("%s: Pick up succeeded!", self.node_name)
                    else:
                        self.state = 2
                        rospy.loginfo("%s: Pick up failed!", self.node_name)

                    rospy.sleep(1)

                except rospy.ServiceException as e:
                    print("Service call to move_head server failed: %s"%e)


            # State 6:  Navigate to place pose
            if self.state == 6:
                self.prev_state = self.state
                
                if is_reading_table_pose:
                    target_pose = GetModelPose("table_3_clone")
                    target_pose.header.frame_id = "map"
                    target_pose.pose.position.y -= 0.7
                else:
                    target_pose = rospy.wait_for_message(self.place_pose_topic_ns, PoseStamped, 5)
                success = self.navigate(target_pose)
                
                # next state
                self.state = 7 if success else -1
                rospy.sleep(1)


            # State 7:  Place service
            if self.state == 7:
                try:
                    rospy.loginfo("%s: Place object", self.node_name)
                    place_service = rospy.ServiceProxy(self.place_service_ns, SetBool)
                    place_req = place_service()
                    self.prev_state = self.state

                    if place_req.success:
                        self.state = 0
                        rospy.loginfo("%s: Place succeeded!", self.node_name)
                    else:
                        self.state = 2
                        rospy.loginfo("%s: Place failed!", self.node_name)

                    rospy.sleep(1)

                except rospy.ServiceException as e:
                    print("Service call to move_head server failed: %s"%e)

        rospy.loginfo("%s: State machine finished!", self.node_name)
        return

	

if __name__ == "__main__":

	rospy.init_node('main_state_machine')
	try:
		StateMachine()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
