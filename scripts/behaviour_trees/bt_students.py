#!/usr/bin/env python3

from behaviours import *

# Visualize py_trees:
# > rosrun rqt_py_trees rqt_py_trees

# Inspect blackboard/tree in command line:
# > py-trees-blackboard-watcher
# > py-trees-tree-watcher
# See https://py-trees-ros.readthedocs.io/en/devel/modules.html#module-py_trees_ros.programs.tree_watcher

# send rostopic
# rostopic pub /dashboard/scan std_msgs/Empty "{}" -r 1000


def TaskC():
    """
    Create a basic tree and start a 'Topics2BB' work sequence that
    takes the asynchronicity out of subscription.

    Returns:
        :class:`~py_trees.behaviour.Behaviour`: the root of the tree
    """
    root = pt.composites.Parallel("Tutorial")

    topics2bb = pt.composites.Sequence("Topics2BB")
    # topics2bb = pt.composites.Parallel("Topics2BB")
    pt.Blackboard().set(name="retry_tasks", value=True)
    pt.Blackboard().set(name="vel_pub_rate", value=10)
    pt.Blackboard().set(name="get_table_pose", value=False)
    pt.Blackboard().set(name="table_name", value="table_3_clone")
    pt.Blackboard().set(name="aruco_cube_counter", value=0)
	
    aruco_to_bb = ptr.subscribers.ToBlackboard(name="aruco_pose",
                                            topic_name=rospy.get_param(rospy.get_name() + '/aruco_pose_topic'),
                                            topic_type=PoseStamped,
                                            blackboard_variables={"aruco_pose": None},
                                            clearing_policy=pt.common.ClearingPolicy.ON_INITIALISE
                                            )
	
    pick_pose_to_bb = ptr.subscribers.ToBlackboard(name="pick_pose",
                                            topic_name=rospy.get_param(rospy.get_name() + '/pick_pose_topic'),
                                            topic_type=PoseStamped,
                                            blackboard_variables={"pick_pose": None},
                                            clearing_policy=pt.common.ClearingPolicy.ON_INITIALISE
                                            )
	
    place_pose_to_bb = ptr.subscribers.ToBlackboard(name="place_pose",
                                            topic_name=rospy.get_param(rospy.get_name() + '/place_pose_topic'),
                                            topic_type=PoseStamped,
                                            blackboard_variables={"place_pose": None},
                                            clearing_policy=pt.common.ClearingPolicy.ON_INITIALISE
                                            )

    robot_pose_to_bb = ptr.subscribers.ToBlackboard(name="robot_pose",
                                            topic_name=rospy.get_param(rospy.get_name() + '/robot_pose_topic'),
                                            topic_type=PoseWithCovarianceStamped,
                                            blackboard_variables={"robot_pose": None},
                                            clearing_policy=pt.common.ClearingPolicy.ON_INITIALISE
                                            )

    priorities = pt.composites.Selector("Priorities")
    
    tasks = pt.composites.Sequence(name="Default tasks")
    is_safe = pt.blackboard.CheckBlackboardVariable(
        name="Retry?",
        variable_name='retry_tasks',
        expected_value=True
    )
    preempt = pt.composites.Selector(name="Preempt?")
    is_safe_cp = pt.meta.success_is_running(pt.blackboard.CheckBlackboardVariable)(
        name="Retry?",
        variable_name='retry_tasks',
        expected_value=True
    )

    pick_and_place_tasks = pt.composites.Sequence(name="Pick and place")

    pick_tasks = pt.composites.Sequence(name="Pick tasks")

    place_tasks = pt.composites.Sequence(name="Place tasks")

    exit_fallback = pt.composites.Selector(name="Exit program")

    exit_program = ExitProgram()

    reset_robot = ResetRobot()

    pause = pt.timers.Timer("Pause", duration=1.0)
    
    tuck_arm = TuckArm()

    reverse = Go("Reverse", linear=-0.05, angular=0, max_ticks=30)

    # move to chair
    move_to_pickup = GoTo("pick")

    # lower head
    
    head_down = pt.composites.Sequence(
        name="Lower robot head",               
        children=[MoveRobotHead("down"), pause]
    )

    find_aruco = pt.composites.Selector(
        name="Find aruco cube fallback",
        children=[LookForAruco(), Go("Spin to find aruco", linear=0, angular=0.4, max_ticks=150)]
    )

    is_placed = pt.composites.Selector(
        name="Find aruco cube fallback",
        children=[LookForAruco(), RetryTasks()]
    )

    arm_pickup = MoveRobotArm("pick")

    move_to_place = GoTo("place")

    arm_place = MoveRobotArm("place")

    # cleanup = pt.composites.Parallel(name="Cleanup", policy=pt.common.ParallelPolicy.SUCCESS_ON_ONE)
    
    root.add_children([topics2bb, priorities])
    topics2bb.add_children([robot_pose_to_bb, pick_pose_to_bb, place_pose_to_bb, aruco_to_bb])
    priorities.add_children([tasks])
    tasks.add_children([exit_fallback, preempt])
    preempt.add_children([pick_and_place_tasks])
    pick_tasks.add_children([move_to_pickup, head_down, find_aruco, arm_pickup, pause])
    place_tasks.add_children([move_to_place, arm_place, pause])
    pick_and_place_tasks.add_children([reset_robot, reverse, tuck_arm, pick_tasks, place_tasks, is_placed])
    exit_fallback.add_children([is_safe, exit_program])

    return root


def TaskA():
    """
    Create a basic tree and start a 'Topics2BB' work sequence that
    takes the asynchronicity out of subscription.

    Returns:
        :class:`~py_trees.behaviour.Behaviour`: the root of the tree
    """
    root = pt.composites.Parallel("Tutorial")

    # topics2bb = pt.composites.Sequence("Topics2BB")
    topics2bb = pt.composites.Parallel("Topics2BB")
    pt.Blackboard().set(name="retry_tasks", value=True)
    pt.Blackboard().set(name="vel_pub_rate", value=10)
    pt.Blackboard().set(name="get_table_pose", value=False)
    pt.Blackboard().set(name="table_name", value="table_3_clone")
    pt.Blackboard().set(name="aruco_cube_counter", value=0)

    joints_to_bb = ptr.subscribers.ToBlackboard(name="joint_states",
                                            topic_name=rospy.get_param(rospy.get_name() + '/joint_states_topic'),
                                            topic_type=JointState,
                                            blackboard_variables={"joint_states": None},
                                            clearing_policy=pt.common.ClearingPolicy.NEVER
                                            )
	
    aruco_to_bb = ptr.subscribers.ToBlackboard(name="aruco_pose",
                                            topic_name=rospy.get_param(rospy.get_name() + '/aruco_pose_topic'),
                                            topic_type=PoseStamped,
                                            blackboard_variables={"aruco_pose": None},
                                            clearing_policy=pt.common.ClearingPolicy.NEVER
                                            )
	
    pick_pose_to_bb = ptr.subscribers.ToBlackboard(name="pick_pose",
                                            topic_name=rospy.get_param(rospy.get_name() + '/pick_pose_topic'),
                                            topic_type=PoseStamped,
                                            blackboard_variables={"pick_pose": None},
                                            clearing_policy=pt.common.ClearingPolicy.NEVER
                                            )
	
    place_pose_to_bb = ptr.subscribers.ToBlackboard(name="place_pose",
                                            topic_name=rospy.get_param(rospy.get_name() + '/place_pose_topic'),
                                            topic_type=PoseStamped,
                                            blackboard_variables={"place_pose": None},
                                            clearing_policy=pt.common.ClearingPolicy.NEVER
                                            )

    robot_pose_to_bb = ptr.subscribers.ToBlackboard(name="robot_pose",
                                            topic_name=rospy.get_param(rospy.get_name() + '/robot_pose_topic'),
                                            topic_type=PoseWithCovarianceStamped,
                                            blackboard_variables={"robot_pose": None},
                                            clearing_policy=pt.common.ClearingPolicy.NEVER
                                            )
    
    scan_to_bb = ptr.subscribers.ToBlackboard(name="scan",
                                            topic_name="/scan_raw",
                                            topic_type=LaserScan,
                                            blackboard_variables={"scan": None},
                                            clearing_policy=pt.common.ClearingPolicy.NEVER
                                            )

    priorities = pt.composites.Selector("Priorities")
    
    tasks = pt.composites.Sequence(name="Default tasks")
    is_safe = pt.blackboard.CheckBlackboardVariable(
        name="Retry?",
        variable_name='retry_tasks',
        expected_value=True
    )
    preempt = pt.composites.Selector(name="Preempt?")
    is_safe_cp = pt.meta.success_is_running(pt.blackboard.CheckBlackboardVariable)(
        name="Retry?",
        variable_name='retry_tasks',
        expected_value=True
    )

    initial_tasks = pt.composites.Sequence(name="Initialise")

    tasks_cp = pt.composites.Parallel(name="Robot tasks")

    pick_and_place_tasks = pt.composites.Sequence(name="Pick and place")

    pick_tasks = pt.composites.Sequence(name="Pick tasks")

    place_tasks = pt.composites.Sequence(name="Place tasks")

    exit_fallback = pt.composites.Selector(name="Exit program")

    retry_tasks = RetryTasks()

    exit_program = ExitProgram()

    reset_robot = ResetRobot()

    pause = pt.timers.Timer("Pause", duration=1.0)
    
    tuck_arm = TuckArm()

    reverse = Go("Reverse", linear=-0.05, angular=0, max_ticks=30)

    localize = Relocalise(linear=0.1, angular=0.8, max_ticks=100)

    localize_cp = Relocalise(linear=0.1, angular=0.8, max_ticks=100)

    # move to chair
    move_to_pickup = GoTo("pick")

    # lower head
    
    head_down = pt.composites.Sequence(
        name="Lower robot head",               
        children=[MoveRobotHead("down"), pause]
    )

    find_aruco = pt.composites.Selector(
        name="Find aruco cube fallback",
        children=[LookForAruco(), Go("Spin to find aruco", linear=0, angular=0.4, max_ticks=150)]
    )

    arm_pickup = MoveRobotArm("pick")

    move_to_place = pt.meta.success_is_running(pt.composites.Selector)(
        name="Move to place pose fallback",
        children=[
            GoTo("place"),
            pt.composites.Selector(name="Check aruco fallback", children=([CheckAruco(), RetryTasks()])),
            ]
    )

    arm_place = MoveRobotArm("place")

    detect_kidnap = DetectKidnap()

    # cleanup = pt.composites.Parallel(name="Cleanup", policy=pt.common.ParallelPolicy.SUCCESS_ON_ONE)
    

    root.add_children([topics2bb, priorities])
    topics2bb.add_children([scan_to_bb, pick_pose_to_bb, place_pose_to_bb, robot_pose_to_bb, aruco_to_bb, joints_to_bb])
    priorities.add_children([tasks])
    tasks.add_children([initial_tasks, exit_fallback, preempt])
    preempt.add_children([tasks_cp])
    tasks_cp.add_children([pick_and_place_tasks, detect_kidnap])
    initial_tasks.add_children([localize, tuck_arm])
    pick_tasks.add_children([move_to_pickup, head_down, find_aruco, arm_pickup])
    place_tasks.add_children([move_to_place, arm_place])
    pick_and_place_tasks.add_children([reset_robot, pick_tasks, place_tasks, retry_tasks])
    exit_fallback.add_children([is_safe, exit_program])
    # cleanup.add_children([pause])
    return root


def shutdown(behaviour_tree):
    behaviour_tree.interrupt()


def main():
    import functools, sys
    rospy.init_node("behaviour_tree")
    root = TaskC()
    behaviour_tree = ptr.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))
    if not behaviour_tree.setup(timeout=15):
        pt.console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)
    behaviour_tree.tick_tock(1)


if __name__ == "__main__":
    main()
