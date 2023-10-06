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
from gazebo_utils import *


class Go(pt.behaviour.Behaviour):

    """
    Returns running and commands a velocity indefinitely.
    """

    def __init__(self, name, linear, angular, max_ticks):

        self.name = name

        self.linear = linear
        self.angular = angular

        self.ticks = 0
        self.max_ticks = max_ticks

        # become a behaviour
        super(Go, self).__init__(name)


    def update(self):

        self.ticks += 1
        if self.ticks < self.max_ticks:
           # send the message 
            self.cmd_vel_pub.publish(self.move_msg)
            rate = pt.Blackboard().get("vel_pub_rate")
            rospy.Rate(rate).sleep()

            # tell the tree that you're running
            return pt.common.Status.RUNNING
        else:
            return pt.common.Status.SUCCESS

        
    def initialise(self):

        rospy.loginfo("Initialising %s behaviour.", self.name)
        # action space
        cmd_vel_topic_ns = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic_ns, Twist, queue_size=10)

        # command
        self.move_msg = Twist()
        self.move_msg.linear.x = self.linear
        self.move_msg.angular.z = self.angular

        self.ticks = 0

        return super().initialise()


    def terminate(self, new_status):
        rospy.loginfo("%s: Terminate!", self.name)
        return super().terminate(new_status)
        

class GoTo(pt.behaviour.Behaviour):

    """
    Sends a goal to the move base action server.
    Returns running whilst awaiting the result,
    success if the action was successful, and v.v..
    """

    def __init__(self, target_name="pick"):

        self.name = "Go to pickup pose" if target_name == "pick" else "Go to place pose"
        self.target_name = target_name
        self.goal = None

        # become a behaviour
        super(GoTo, self).__init__(self.name)


    def update(self):

        # if success:
        if self.move_base_ac.get_result():
            rospy.loginfo("%s: Success!", self.name)
            return pt.common.Status.SUCCESS

        else:
            return pt.common.Status.RUNNING


    def initialise(self):
        # setup action client
        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)
        if not self.move_base_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("Could not connect to move_base action server.")
            exit()

        # personal goal setting
        self.goal = MoveBaseGoal()
        target_pose = pt.Blackboard().get(self.target_name+"_pose")
        use_table_pose = pt.Blackboard().get("get_table_pose")
        if use_table_pose and self.target_name is not "pick":
            table_name = pt.Blackboard().get("table_name")
            table_pose = GetModelPose(table_name)
            target_pose.pose = table_pose.pose
            target_pose.pose.position.y -= 0.6

        self.goal.target_pose = target_pose
        rospy.loginfo(target_pose)
        self.move_base_ac.send_goal(self.goal)

        return super().initialise()


    def terminate(self, new_status):
        rospy.loginfo("Initialising %s behaviour.", self.name)
        if self.goal is not None:
            self.move_base_ac.cancel_goal()  
        return super().terminate(new_status)


class TuckArm(pt.behaviour.Behaviour):

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
        self.tried = False
        self.finished = False

        # become a behaviour
        super(TuckArm, self).__init__("Tuck arm")


    def update(self):

        # already tucked the arm
        if self.finished:
            return pt.common.Status.SUCCESS
        
        # command to tuck arm if haven't already
        if not self.tried:
            rospy.loginfo("%s: Start!", self.name)
            # send the goal
            self.play_motion_ac.send_goal(self.goal)
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.play_motion_ac.get_result():
            rospy.loginfo("%s: Success!", self.name)
            self.finished = True
            return pt.common.Status.SUCCESS

        # # if failed
        # elif not self.play_motion_ac.get_result():
        #     rospy.loginfo("%s: Failed!", self.name)
        #     self.finished = True
        #     return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            # rospy.Rate(rate).sleep()
            return pt.common.Status.RUNNING


    def initialise(self):
        return super().initialise()
    

    def terminate(self, new_status):
        return super().terminate(new_status)


class MoveRobotHead(pt.behaviour.Behaviour):

    """
    Lowers or raises the head of the robot.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self, direction):

        self.name = "Lower head" if direction == "down" else "Raise head"
        # head movement direction; "down" or "up"
        self.direction = direction

        # become a behaviour
        super(MoveRobotHead, self).__init__(self.name)


    def update(self):

        # if succesful
        if self.move_head_req.success:
            rospy.loginfo("%s: Success!", self.name)
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.move_head_req.success:
            rospy.loginfo("%s: Failed!", self.name)
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING


    def initialise(self):
        rospy.loginfo("Initialising move head behaviour.")

        # server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        self.move_head_req = self.move_head_srv(self.direction)

        return super().initialise()
        

    def terminate(self, new_status):
        return super().terminate(new_status)


class MoveRobotArm(pt.behaviour.Behaviour):

    """
    Pick or place the aruco cude using robot arm.
    Returns running whilst awaiting the result,
    success if the action was successful, and v.v..
    """

    def __init__(self, direction):

        self.name = "Pick up aruco cube" if direction == "pick" else "Place aruco cube"

        # arm movement direction; "pick" or "place"
        self.direction = direction

        # become a behaviour
        super(MoveRobotArm, self).__init__(self.name)


    def update(self):

        # if successful
        if self.move_arm_req.success:
            rospy.loginfo("%s: Success!", self.name)
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.move_arm_req.success:
            rospy.loginfo("%s: Failed!", self.name)
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING


    def initialise(self):
        rospy.loginfo("Initialising %s behaviour.", self.name)

        # server
        move_arm_service_ns = rospy.get_param('%s/%s_srv' % (rospy.get_name(), self.direction))
        self.move_arm_service = rospy.ServiceProxy(move_arm_service_ns, SetBool)
        rospy.wait_for_service(move_arm_service_ns, timeout=30)

        self.move_arm_req = self.move_arm_service()

        return super().initialise()


    def terminate(self, new_status):
        return super().terminate(new_status)


class LookForAruco(pt.behaviour.Behaviour):

    """
    Clear both local and global cost map.
    Returns running whilst awaiting the result,
    success if the action was successful, and v.v..
    """

    def __init__(self):

        self.name = "Detect aruco cube"

        # become a behaviour
        super(LookForAruco, self).__init__(self.name)


    def update(self):
        aruco_pose_msg = pt.Blackboard().get("aruco_pose")
        if type(aruco_pose_msg) is PoseStamped and aruco_pose_msg.pose.position.z > 0:
            rospy.loginfo("%s: Success!", self.name)
            return pt.common.Status.SUCCESS

        else:
            rospy.loginfo("%s: Failed!", self.name)
            return pt.common.Status.FAILURE


    def initialise(self):
        rospy.loginfo("Initialising detect aruco behaviour.")

        return super().initialise()


    def terminate(self, new_status):
        return super().terminate(new_status)


class CheckAruco(pt.behaviour.Behaviour):

    """
    Clear both local and global cost map.
    Returns running whilst awaiting the result,
    success if the action was successful, and v.v..
    """

    def __init__(self):

        self.name = "Check aruco cube"

        # execution checker
        self.disp_threshold = 0.02
        self.force_threshold = 0.5

        # become a behaviour
        super(CheckAruco, self).__init__(self.name)


    def update(self):

        states = pt.Blackboard().get("joint_states")
        p_left, p_right = states.position[7], states.position[8]
        f_left, f_right = states.effort[7], states.effort[8]
        form_closed = (p_left < self.disp_threshold) and (p_right < self.disp_threshold)
        force_reduced = (abs(f_left) < self.force_threshold) and (abs(f_right) < self.force_threshold)

        if form_closed and force_reduced:
            rospy.loginfo("Aruco cube is missing!")
            rospy.loginfo("Position: %0.2f, %0.2f", p_left, p_right)
            rospy.loginfo("Effort: %0.2f, %0.2f", f_left, f_right)

            # tell the tree that you're running
            rospy.loginfo("%s: Failed!", self.name)
            self.finished = True
            return pt.common.Status.SUCCESS
        else:
            rospy.loginfo("%s: Success!", self.name)
            self.finished = True
            return pt.common.Status.FAILURE


    def initialise(self):
        rospy.loginfo("Initialising %s behaviour.", self.name)
        self.disp_threshold = 0.02
        self.force_threshold = 0.5
        return super().initialise()


class ResetRobot(pt.behaviour.Behaviour):

    """
    Clear both local and global cost map.
    Returns running whilst awaiting the result,
    success if the action was successful, and v.v..
    """

    def __init__(self):

        self.name = "Reset robot"
        # become a behaviour
        super(ResetRobot, self).__init__(self.name)


    def update(self):
        if self.move_head_req.success:
            rospy.loginfo("%s: Success!", self.name)
            return pt.common.Status.SUCCESS
        else: 
            return pt.common.Status.RUNNING


    def initialise(self):
        rospy.loginfo("Initialising robot status...")

        ResetAruco()

        # server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        self.move_head_req = self.move_head_srv("up")
        
        return super().initialise()
    
    def terminate(self, new_status):
        return super().terminate(new_status)


class resettasks(pt.behaviour.Behaviour):

    """
    Clear both local and global cost map.
    Returns running whilst awaiting the result,
    success if the action was successful, and v.v..
    """

    def __init__(self):

        self.name = "Reset tasks"
        
        # become a behaviour
        super(resettasks, self).__init__(self.name)


    def update(self):
        pt.Blackboard().set("reset_tasks", False)
        return pt.common.Status.SUCCESS


class exitseq(pt.behaviour.Behaviour):

    def __init__(self):

        self.name = "Exit"
        rospy.loginfo("Initialising exit sequence behaviour.")

        # become a behaviour
        super(exitseq, self).__init__(self.name)


    def update(self):
        exit()

