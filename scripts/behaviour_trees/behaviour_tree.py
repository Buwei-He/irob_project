#!/usr/bin/env python3

from behaviours_student import *

# Visualize py_trees:
# > rosrun rqt_py_trees rqt_py_trees

# Inspect blackboard/tree in command line:
# > py-trees-blackboard-watcher
# > py-trees-tree-watcher
# See https://py-trees-ros.readthedocs.io/en/devel/modules.html#module-py_trees_ros.programs.tree_watcher

# send rostopic
# rostopic pub /dashboard/scan std_msgs/Empty "{}" -r 1000


def create_root():
    """
    Create a basic tree and start a 'Topics2BB' work sequence that
    takes the asynchronicity out of subscription.

    Returns:
        :class:`~py_trees.behaviour.Behaviour`: the root of the tree
    """
    root = pt.composites.Parallel("Tutorial")

    topics2bb = pt.composites.Sequence("Topics2BB")
    pt.Blackboard().set(name="reset_tasks", value=True)
    pt.Blackboard().set(name="vel_pub_rate", value=10)
    pt.Blackboard().set(name="get_table_pose", value=True)
    pt.Blackboard().set(name="table_name", value="table_3_clone")

    joints2bb = ptr.subscribers.ToBlackboard(name="joint_states",
                                            topic_name=rospy.get_param(rospy.get_name() + '/joint_states_topic'),
                                            topic_type=JointState,
                                            blackboard_variables={"joint_states": None},
                                            clearing_policy=pt.common.ClearingPolicy.NEVER
                                            )
	
    aruco2bb = ptr.subscribers.ToBlackboard(name="aruco_pose",
                                            topic_name=rospy.get_param(rospy.get_name() + '/aruco_pose_topic'),
                                            topic_type=PoseStamped,
                                            blackboard_variables={"aruco_pose": None},
                                            clearing_policy=pt.common.ClearingPolicy.NEVER
                                            )
	
    pick2bb = ptr.subscribers.ToBlackboard(name="pick_pose",
                                            topic_name=rospy.get_param(rospy.get_name() + '/pick_pose_topic'),
                                            topic_type=PoseStamped,
                                            blackboard_variables={"pick_pose": None},
                                            clearing_policy=pt.common.ClearingPolicy.NEVER
                                            )
	
    place2bb = ptr.subscribers.ToBlackboard(name="place_pose",
                                            topic_name=rospy.get_param(rospy.get_name() + '/place_pose_topic'),
                                            topic_type=PoseStamped,
                                            blackboard_variables={"place_pose": None},
                                            clearing_policy=pt.common.ClearingPolicy.NEVER
                                            )

    priorities = pt.composites.Selector("Priorities")
    
    tasks = pt.composites.Sequence(name="Tasks")
    is_safe = pt.blackboard.CheckBlackboardVariable(
        name="Safe?",
        variable_name='reset_tasks',
        expected_value=True
    )
    preempt = pt.composites.Selector(name="Preempt?")
    is_safe_cp = pt.meta.success_is_running(pt.blackboard.CheckBlackboardVariable)(
        name="Safe?",
        variable_name='reset_tasks',
        expected_value=True
    )
    pick_and_place = pt.composites.Sequence(name="Pick and place")

    reset_robot = ResetRobot()
    
    tuck_arm = ptr.actions.ActionClient(name="Tuck arm",
                                        action_spec=PlayMotionAction,
                                        action_goal=PlayMotionGoal(motion_name='home', skip_planning=True),
                                        action_namespace="/play_motion")

    reverse = Go("Reverse", linear=-0.2, angular=0, max_ticks=20)

    localize = Go("Spin to localize", linear=0, angular=-1, max_ticks=60)

    # move to chair
    move_to_pickup = GoTo("pick")

    # lower head
    head_down = MoveRobotHead("down")

    find_aruco = pt.composites.Selector(
        name="Find aruco cube fallback",
        children=[LookForAruco(), Go("Spin to find aruco", linear=0, angular=0.2, max_ticks=300)]
    )

    arm_pickup = MoveRobotArm("pick")

    move_to_place = pt.composites.Selector(
        name="move to place pose fallback",
        children=[CheckAruco(), GoTo("place")])

    arm_place = MoveRobotArm("place")

    cleanup = pt.composites.Parallel(name="Cleanup", policy=pt.common.ParallelPolicy.SUCCESS_ON_ONE)
    pause = pt.timers.Timer("Pause", duration=3.0)
    exit_program = exitseq()

    root.add_children([topics2bb, priorities])
    topics2bb.add_children([pick2bb, place2bb, aruco2bb, joints2bb])
    priorities.add_children([tasks])
    tasks.add_children([is_safe, preempt])
    preempt.add_children([pick_and_place])
    pick_and_place.add_children([reverse, tuck_arm, localize, reset_robot, move_to_pickup, head_down, find_aruco, arm_pickup, move_to_place, arm_place])
    cleanup.add_children([pause])
    return root


# become the tree

def shutdown(behaviour_tree):
    behaviour_tree.interrupt()


def main():
    import functools, sys
    rospy.init_node("behaviour_tree")
    root = create_root()
    behaviour_tree = ptr.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))
    if not behaviour_tree.setup(timeout=15):
        pt.console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)
    behaviour_tree.tick_tock(5)


if __name__ == "__main__":
    main()
