# Christopher Iliffe Sprague
# sprague@kth.se
# Behaviours needed for the example student solution.


import py_trees as pt, py_trees_ros as ptr, rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped
from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from actionlib import SimpleActionClient, SimpleGoalState
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import JointState
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse


class counter(pt.behaviour.Behaviour):

    """
    Returns running for n ticks and success thereafter.
    """

    def __init__(self, n, name):

        rospy.loginfo("Initialising counter behaviour.")

        # counter
        self.i = 0
        self.n = n

        # become a behaviour
        super(counter, self).__init__(name)


    def update(self):

        # increment i
        self.i += 1

        # succeed after count is done
        return pt.common.Status.FAILURE if self.i <= self.n else pt.common.Status.SUCCESS


class go(pt.behaviour.Behaviour):

    """
    Returns running and commands a velocity indefinitely.
    """

    def __init__(self, name, linear, angular):

        rospy.loginfo("Initialising goto behaviour.")

        # action space
        self.cmd_vel_topic_ns = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic_ns, Twist, queue_size=10)

        # command
        self.move_msg = Twist()
        self.move_msg.linear.x = linear
        self.move_msg.angular.z = angular

        # become a behaviour
        super(go, self).__init__(name)


    def update(self):

        # send the message
        rate = rospy.Rate(10)
        self.cmd_vel_pub.publish(self.move_msg)
        rate.sleep()
        rospy.loginfo("Moving...")

        # tell the tree that you're running
        return pt.common.Status.RUNNING


class goto(pt.behaviour.Behaviour):

    """
    Sends a goal to the move base action server.
    Returns running whilst awaiting the result,
    success if the action was successful, and v.v..
    """

    def __init__(self, target_pose="pick", with_aruco=False):

        rospy.loginfo("Initialising goto behaviour.")
        self.name = "Goto pickup pose!" if target_pose == "pick" else "Goto place pose!"

        # setup action client
        target_pose_topic = rospy.get_param('%s/%s_pose_topic' % (rospy.get_name(), target_pose))
        target_pose = rospy.wait_for_message(target_pose_topic, PoseStamped, 5)

        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)
        if not self.move_base_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("Could not connect to move_base action server.")
            exit()
        rospy.loginfo("Connected to move_base action server.")

        # personal goal setting
        self.goal = MoveBaseGoal()
        self.goal.target_pose = target_pose

        # aruco cube detect
        self.gripper_position = [1, 1]
        self.gripper_effort = [1, 1]
        self.with_aruco = with_aruco
        if with_aruco:
            self.joint_states_topic_ns = rospy.get_param(rospy.get_name() + '/joint_states_topic')
            self.joint_states_subs = rospy.Subscriber(self.joint_states_topic_ns, JointState, self.joint_states_callback)

        # execution checker
        self.sent_goal = False
        self.finished = False

        # become a behaviour
        super(goto, self).__init__(self.name)


    def joint_states_callback(self, joint_states_msg):
        """ Update gripper joints state. """
        valid = joint_states_msg is not None
        if valid:
            self.gripper_position = [joint_states_msg.position[7], joint_states_msg.position[8]]
            self.gripper_effort = [joint_states_msg.effort[7], joint_states_msg.effort[8]]
        return valid


    def gripping_aruco(self):
        # gripper distance and effort (force)
        disp_threshold = 0.02
        force_threshold = 0.5
        p_left, p_right = self.gripper_position[0], self.gripper_position[1]
        f_left, f_right = self.gripper_effort[0], self.gripper_effort[1]
        form_closed = (p_left < disp_threshold) and (p_right < disp_threshold)
        force_reduced = (abs(f_left) < force_threshold) and (abs(f_right) < force_threshold)
        # visual detect?
        if self.with_aruco and form_closed and force_reduced:
            rospy.loginfo("Aruco cube is missing!")
            rospy.loginfo("Position: %0.2f, %0.2f", p_left, p_right)
            rospy.loginfo("Effort: %0.2f, %0.2f", f_left, f_right)
            return False
        
        else: return True


    def update(self):

        # Already finished?
        if self.finished: 
            return pt.common.Status.SUCCESS
        
        # Command to do something if haven't already...
        elif not self.sent_goal:

            # send the goal
            self.move_base_ac.send_goal(self.goal)
            self.sent_goal = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if success:
        elif self.move_base_ac.get_result():

            # than I'm finished!
            rospy.loginfo("Goto target pose: Success!")
            self.finished = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.move_base_ac.get_result() and self.move_base_ac.get_state() == SimpleGoalState.DONE:
            rospy.loginfo("Goto target pose: Failed!")
            self.move_base_ac.cancel_goal()
            self.sent_goal = False
            return pt.common.Status.FAILURE
        
        # if dropped
        elif not self.gripping_aruco():
            rospy.loginfo("Hej, aruco cube is missing!")
            self.move_base_ac.cancel_goal()
            self.sent_goal = False
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING
        

class tuckarm(pt.behaviour.Behaviour):

    """
    Sends a goal to the tuck arm action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        rospy.loginfo("Initialising tuck arm behaviour.")

        # Set up action client
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

        # personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'home'
        self.goal.skip_planning = True

        # execution checker
        self.sent_goal = False
        self.finished = False

        # become a behaviour
        super(tuckarm, self).__init__("Tuck arm!")


    def update(self):

        # already tucked the arm
        if self.finished: 
            return pt.common.Status.SUCCESS
        
        # command to tuck arm if haven't already
        elif not self.sent_goal:

            # send the goal
            self.play_motion_ac.send_goal(self.goal)
            self.sent_goal = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.play_motion_ac.get_result():

            # than I'm finished!
            self.finished = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.play_motion_ac.get_result():
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING


class movehead(pt.behaviour.Behaviour):

    """
    Lowers or raises the head of the robot.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self, direction):

        self.name = "Lower head!" if direction == "down" else "Raise head!"
        rospy.loginfo("Initialising move head behaviour.")

        # server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        # head movement direction; "down" or "up"
        self.direction = direction

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(movehead, self).__init__(self.name)


    def update(self):

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.move_head_req = self.move_head_srv(self.direction)
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.move_head_req.success:
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.move_head_req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING


class movearm(pt.behaviour.Behaviour):

    """
    Pick or place the aruco cude using robot arm.
    Returns running whilst awaiting the result,
    success if the action was successful, and v.v..
    """

    def __init__(self, direction):

        self.name = "Pick up aruco cube!" if direction == "pick" else "Place aruco cube!"
        rospy.loginfo("Initialising move head behaviour.")

        # server
        move_arm_service_ns = rospy.get_param('%s/%s_srv' % (rospy.get_name(), direction))
        self.move_arm_service = rospy.ServiceProxy(move_arm_service_ns, SetBool)
        rospy.wait_for_service(move_arm_service_ns, timeout=30)

        # arm movement direction; "pick" or "place"
        self.direction = direction

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(movearm, self).__init__(self.name)


    def update(self):

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.move_arm_req = self.move_arm_service()
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if successful
        elif self.move_arm_req.success:
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.move_arm_req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING


class clearcost(pt.behaviour.Behaviour):

    """
    Clear both local and global cost map.
    Returns running whilst awaiting the result,
    success if the action was successful, and v.v..
    """

    def __init__(self):

        self.name = "Clear cost map!"
        rospy.loginfo("Initialising move head behaviour.")

        # server
        clear_costmaps_service_ns = rospy.get_param(rospy.get_name() + '/clear_costmaps_srv')
        self.clear_costmap_service = rospy.ServiceProxy(self.clear_costmaps_service_ns, Empty)
        rospy.wait_for_service(clear_costmaps_service_ns, timeout=30)

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(clearcost, self).__init__(self.name)


    def update(self):

        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.clear_costmap_req = self.clear_costmap_service()
            self.tried = True
            self.done = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if still trying
        else:
            return pt.common.Status.RUNNING


class lookforaruco(pt.behaviour.Behaviour):

    """
    Clear both local and global cost map.
    Returns running whilst awaiting the result,
    success if the action was successful, and v.v..
    """

    def __init__(self, angular):

        self.name = "Detect aruco cube!"
        rospy.loginfo("Initialising detect aruco behaviour.")

        # velocity topic
        self.cmd_vel_topic_ns = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic_ns, Twist, queue_size=10)

        # command
        self.move_msg = Twist()
        self.move_msg.angular.z = angular

        # subscribe to vision-based detector
        self.aruco_pose_topic_ns = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
        self.aruco_pose_subs = rospy.Subscriber(self.aruco_pose_topic_ns, PoseStamped, self.aruco_pose_callback)

        rospy.loginfo("Looking for aruco cube...")

        # execution checker
        self.aruco_detected = False

        # become a behaviour
        super(lookforaruco, self).__init__(self.name)


    def aruco_pose_callback(self, aruco_pose_msg):
        """ Update aruco pose. """
        valid = aruco_pose_msg is not None
        if valid:
            self.aruco_detected = True
        return valid


    def update(self):

        # spin if aruco is not detected
        if not self.aruco_detected:

            # send the message
            rate = rospy.Rate(10)
            self.cmd_vel_pub.publish(self.move_msg)
            rate.sleep()

            # tell the tree that you're running
            return pt.common.Status.RUNNING
        
        else: return pt.common.Status.SUCCESS


class resetaruco(pt.behaviour.Behaviour):

    """
    Clear both local and global cost map.
    Returns running whilst awaiting the result,
    success if the action was successful, and v.v..
    """

    def __init__(self):

        self.name = "Reset aruco cube!"
        rospy.loginfo("Initialising reset aruco behaviour.")

        # become a behaviour
        super(resetaruco, self).__init__(self.name)

    def update(self):

        from reset_aruco import reset_aruco
        reset_aruco()
        rospy.loginfo("Reset aruco cube!")

        # tell the tree that you're success
        return pt.common.Status.SUCCESS
