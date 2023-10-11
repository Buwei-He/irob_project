# Christopher Iliffe Sprague
# sprague@kth.se
# Behaviours needed for the example student solution.


import py_trees as pt, py_trees_ros as ptr, rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovarianceStamped
from std_srvs.srv import Empty, SetBool, SetBoolRequest  
from actionlib import SimpleActionClient, SimpleGoalState
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from sensor_msgs.msg import JointState, LaserScan
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse
from std_msgs.msg import Bool
from gazebo_utils import *


class Go(pt.behaviour.Behaviour):

    """
    Returns running and commands a velocity indefinitely.
    """

    def __init__(self, name, linear, angular, max_ticks):

        self.name = name
        
        # action space
        cmd_vel_topic_ns = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic_ns, Twist, queue_size=10)

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
        
        # command
        self.move_msg = Twist()
        self.move_msg.linear.x = self.linear
        self.move_msg.angular.z = self.angular
        self.ticks = 0

        return super().initialise()


    def terminate(self, new_status):
        rospy.loginfo("%s: Terminate!", self.name)
        self.cmd_vel_pub.publish(Twist())
        return super().terminate(new_status)
        

class GoTo(pt.behaviour.Behaviour):

    """PoseStamped
    """

    def __init__(self, target_name="pick"):

        self.name = "Go to pickup pose" if target_name == "pick" else "Go to place pose"
        self.target_name = target_name

        # setup action client
        self.move_base_ac = SimpleActionClient("/move_base", MoveBaseAction)
        if not self.move_base_ac.wait_for_server(rospy.Duration(1000)):
            rospy.logerr("Could not connect to move_base action server.")
            exit()

        # personal goal setting
        self.goal = MoveBaseGoal()

        self.tried = False
        # self.amcl_cnt = 0
        # self.amcl_cnt_max = 10
        # self.prev_amcl_sum = 100

        # become a behaviour
        super(GoTo, self).__init__(self.name)


    def update(self):

        # if success:
        if not self.tried:
            self.move_base_ac.send_goal(self.goal)
            self.move_base_ac.wait_for_result(rospy.Duration(100.0))
            self.tried = True

        if self.move_base_ac.get_result():
            rospy.loginfo("%s: Success!", self.name)
            return pt.common.Status.SUCCESS

        else:
            return pt.common.Status.RUNNING


    def initialise(self):
        rospy.loginfo("Initialising %s behaviour.", self.name)
        target_pose = pt.Blackboard().get(self.target_name+"_pose")
        use_table_pose = pt.Blackboard().get("get_table_pose")
        if use_table_pose and self.target_name != "pick":
            table_name = pt.Blackboard().get("table_name")
            target_pose = GetModelPose(table_name)
            target_pose.header.frame_id = "map"
            target_pose.pose.position.y -= 0.95

        self.goal.target_pose = target_pose
        # rospy.loginfo(target_pose)
        
        self.tried = False

        return super().initialise()


    def terminate(self, new_status):
        rospy.loginfo("Terminating %s behaviour.", self.name)
        rospy.loginfo("Result: %s", self.move_base_ac.get_state())
        self.move_base_ac.cancel_all_goals()
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
        self.sent_goal = False
        self.finished = False
        # become a behaviour
        super(TuckArm, self).__init__("Tuck arm!")
        
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
        # # if failed
        # elif not self.play_motion_ac.get_result():
        #     return pt.common.Status.FAILURE
        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING
        
    def initialise(self):
        self.finished = False
        self.sent_goal = False
        return super().initialise()


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
        aruco_pose_seq = pt.Blackboard().get("aruco_pose_seq")
        if type(aruco_pose_msg) is PoseStamped and aruco_pose_msg.header.seq > aruco_pose_seq:
            rospy.loginfo("%s: Success!", self.name)
            return pt.common.Status.SUCCESS

        else:
            # rospy.loginfo("%s: Failed!", self.name)
            return pt.common.Status.FAILURE


    def initialise(self):
        # rospy.loginfo("Initialising %s behaviour.", self.name)
        return super().initialise()


    def terminate(self, new_status):
        return super().terminate(new_status)


class CleanArucoPose(pt.behaviour.Behaviour):

    def __init__(self):
        self.name = "Clean aruco pose"
        # become a behaviour
        super(CleanArucoPose, self).__init__(self.name)

    def initialise(self):
        rospy.loginfo("Initialising %s behaviour.", self.name)
        aruco_pose_msg = pt.Blackboard().get("aruco_pose")
        if type(aruco_pose_msg) is PoseStamped:
            pt.Blackboard().set("aruco_pose_seq", aruco_pose_msg.header.seq)
        else: 
            pt.Blackboard().set("aruco_pose_seq", -1)
        return super().initialise()
    
    def update(self):
        return pt.common.Status.SUCCESS


class CheckAruco(pt.behaviour.Behaviour):

    """
    Clear both local and global cost map.
    Returns running whilst awaiting the result,
    success if the action was successful, and v.v..
    """

    def __init__(self):

        self.name = "Check aruco cube"

        # execution checker
        self.max_ticks = 1000
        self.disp_threshold = 0.02
        self.force_threshold = 0.5

        # become a behaviour
        super(CheckAruco, self).__init__(self.name)


    def update(self):

        states = pt.Blackboard().get("joint_states")
        ticks = pt.Blackboard().get("aruco_cube_counter")
        p_left, p_right = states.position[7], states.position[8]
        f_left, f_right = states.effort[7], states.effort[8]
        form_closed = (p_left < self.disp_threshold) and (p_right < self.disp_threshold)
        force_reduced = (abs(f_left) < self.force_threshold) and (abs(f_right) < self.force_threshold)

        if form_closed and force_reduced:
            ticks += 1
        else:
            ticks = 0
        pt.Blackboard().set("aruco_cube_counter", ticks)

        if not ticks < self.max_ticks:
            rospy.loginfo("Aruco cube is missing!")
            rospy.loginfo("Position: %0.2f, %0.2f", p_left, p_right)
            rospy.loginfo("Effort: %0.2f, %0.2f", f_left, f_right)
            pt.Blackboard().set("retry", True)
            # tell the tree that you're running
            rospy.loginfo("%s: Failed!", self.name)
            return pt.common.Status.FAILURE
        else:
            rospy.loginfo("%s: Success!", self.name)
            return pt.common.Status.SUCCESS


    def initialise(self):
        # rospy.loginfo("Initialising %s behaviour.", self.name)
        self.ticks = 0
        self.max_ticks = 1
        self.disp_threshold = 0.02
        self.force_threshold = 0.5
        return super().initialise()


class Relocalise(pt.behaviour.Behaviour):

    """
    Returns running and commands a velocity indefinitely.
    """

    def __init__(self, linear, angular, max_ticks):

        self.name = "Relocalise"
        
        # action space
        cmd_vel_topic_ns = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic_ns, Twist, queue_size=10)

        self.linear = linear
        self.angular = angular

        self.ticks = 0
        self.max_ticks = max_ticks
        self.max_cov = 0.002

        # become a behaviour
        super(Relocalise, self).__init__(self.name)


    def update(self):

        amcl_pose = pt.Blackboard().get("robot_pose")
        if amcl_pose is not None:
            amcl_cov = amcl_pose.pose.covariance
            is_converged = np.sum(np.abs(amcl_cov)) < self.max_cov
        else: 
            is_converged = False

        if self.ticks < self.max_ticks and not is_converged:
           # send the message 
            self.cmd_vel_pub.publish(self.move_msg)
            rate = pt.Blackboard().get("vel_pub_rate")
            self.ticks += 1
            rospy.Rate(rate).sleep()

            # tell the tree that you're running
            return pt.common.Status.RUNNING
        else:
            return pt.common.Status.SUCCESS

        
    def initialise(self):

        rospy.loginfo("Initialising %s behaviour.", self.name)
        
        # command
        self.move_msg = Twist()
        self.move_msg.linear.x = self.linear
        self.move_msg.angular.z = self.angular
        self.ticks = 0

        global_loc_service_ns = rospy.get_param(rospy.get_name() + '/global_loc_srv')
        self.global_loc_service = rospy.ServiceProxy(global_loc_service_ns, Empty)
        rospy.wait_for_service(global_loc_service_ns, timeout=1)

        self.global_loc_req = self.global_loc_service()

        # reset kidnap status
        pt.Blackboard().set("kidnap", False)

        return super().initialise()


    def terminate(self, new_status):
        rospy.loginfo("%s: Terminate!", self.name)
        self.cmd_vel_pub.publish(Twist())
        return super().terminate(new_status)


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

        # reset task status
        pt.Blackboard().set("retry", False)
        pt.Blackboard().set("kidnap", False)

        # reset aruco cube
        ResetAruco()

        # server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=1)

        self.move_head_req = self.move_head_srv("up")
        
        return super().initialise()
    
    def terminate(self, new_status):
        return super().terminate(new_status)


class ClearCostmap(pt.behaviour.Behaviour):

    """
    Clear both local and global cost map.
    Returns running whilst awaiting the result,
    success if the action was successful, and v.v..
    """

    def __init__(self):

        self.name = "Clear costmap"
        clear_costmap_service_ns = '/move_base/clear_costmaps'
        self.clear_costmap_service = rospy.ServiceProxy(clear_costmap_service_ns, Empty)
        rospy.wait_for_service(clear_costmap_service_ns, timeout=1)

        # become a behaviour
        super(ClearCostmap, self).__init__(self.name)


    def update(self):
        # clear costmap before planning
        rospy.loginfo("Clear costmap!")
        self.clear_costmap_service()
        return pt.common.Status.SUCCESS
    
    
    def initialise(self):
        rospy.loginfo("Initialising %s behaviour.", self.name)
        return super().initialise()


class SetKidnap(pt.behaviour.Behaviour):

    """
    Clear both local and global cost map.
    Returns running whilst awaiting the result,
    success if the action was successful, and v.v..
    """

    def __init__(self, bool):

        self.name = "Kidnap"
        self.bool = bool

        # become a behaviour
        super(SetKidnap, self).__init__(self.name)


    def update(self):
        pt.Blackboard().set("kidnap", self.bool)
        return pt.common.Status.SUCCESS


class SetRetry(pt.behaviour.Behaviour):

    """
    Clear both local and global cost map.
    Returns running whilst awaiting the result,
    success if the action was successful, and v.v..
    """

    def __init__(self, bool):

        self.name = "Retry"
        self.bool = bool
        # self.pub = rospy.Publisher(name="/retry", data_class=Bool, queue_size=1)
        # self.msg = Bool()
        # self.msg.data = True

        # become a behaviour
        super(SetRetry, self).__init__(self.name)


    def update(self):
        pt.Blackboard().set("retry", self.bool)
        # self.pub.publish(self.msg)
        return pt.common.Status.SUCCESS


class DetectKidnap(pt.behaviour.Behaviour):

    """
    Clear both local and global cost map.
    Returns running whilst awaiting the result,
    success if the action was successful, and v.v..
    """

    def __init__(self):

        self.name = "Detect kidnap"
        self.prev_scan : LaserScan = None
        self.prev_pose : PoseWithCovarianceStamped = None
        self.tick_counter = 0
        
        # become a behaviour
        super(DetectKidnap, self).__init__(self.name)


    def get_yaw(self, q):
        return np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


    def update(self):
        curr_scan = pt.Blackboard().get("scan")
        curr_pose = pt.Blackboard().get("robot_pose")

        # rate 10hz
        if self.tick_counter % 10 == 0:
            if self.prev_scan is not None and curr_scan is not None\
                and self.prev_pose is not None and curr_pose is not None:
                prev_yaw = self.get_yaw(self.prev_pose.pose.pose.orientation)
                curr_yaw = self.get_yaw(curr_pose.pose.pose.orientation)
                diff_yaw = (curr_yaw - prev_yaw) % (2*np.pi)

                prev_seq = self.prev_scan.header.seq
                curr_seq = curr_scan.header.seq
                diff_seq = curr_seq - prev_seq
                # diff_yaw_cnt = round(diff_yaw / curr_scan.angle_increment)

                # check length to make sure overlapping part exists; or skip (rotate speed too fast)
                # if abs(diff_yaw_cnt) < 0.8 * len(self.prev_scan.ranges):

                #     if diff_yaw > 0:
                #         prev_ranges = self.prev_scan.ranges[:-diff_yaw_cnt]
                #         curr_ranges = curr_scan.ranges[diff_yaw_cnt:]
                #     elif diff_yaw < 0:
                #         prev_ranges = self.prev_scan.ranges[diff_yaw_cnt:]
                #         curr_ranges = curr_scan.ranges[:-diff_yaw_cnt]
                #     else:
                #         prev_ranges = self.prev_scan.ranges
                #         curr_ranges = curr_scan.ranges
                #     curr_ranges = np.clip(curr_ranges, curr_scan.range_min, curr_scan.range_max)
                #     prev_ranges = np.clip(prev_ranges, self.prev_scan.range_min, self.prev_scan.range_max)

                # simple version
                prev_ranges = self.prev_scan.ranges
                curr_ranges = curr_scan.ranges
                prev_ranges = np.clip(prev_ranges, self.prev_scan.range_min, self.prev_scan.range_max)
                curr_ranges = np.clip(curr_ranges, curr_scan.range_min, curr_scan.range_max)
                

                # calculate cross-correlation of 2 scans
                correlation = np.correlate(curr_ranges, prev_ranges)[0] / (np.linalg.norm(curr_ranges) * np.linalg.norm(prev_ranges))
                # if correlation < 0.999:
                # rospy.loginfo("corr: %.3f", correlation)
                # rospy.loginfo("diff: %.3f", diff_yaw)
                if diff_yaw > 1e-6 and correlation < 0.85:
                    rospy.loginfo("Kidnap?")
                    pt.Blackboard().set("kidnap", True)

            self.prev_pose = curr_pose
            self.prev_scan = curr_scan
        self.tick_counter += 1
        return pt.common.Status.RUNNING


    def initialise(self):
        self.prev_scan = None
        self.prev_pose = None
        self.tick_counter = 0
        return super().initialise()
    

    def terminate(self, new_status):
        return super().terminate(new_status)


class ExitProgram(pt.behaviour.Behaviour):

    def __init__(self):

        self.name = "Exit"

        # become a behaviour
        super(ExitProgram, self).__init__(self.name)


    def update(self):
        rospy.loginfo(" --- End of mission! --- ")
        exit()

