#!/usr/bin/env python3

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		rospy.loginfo("Initialising behaviour tree")

		# go to door until at door
		b0 = pt.composites.Selector(
			name="Go back fallback", 
			children=[counter(15, "Go back done?"), Go("Go back", linear=-0.1, angular=0)]
		)

		# tuck the arm
		b1 = TuckArm()

		# go to table
		b2 = MoveHead("up")

		b3 = pt.composites.Selector(
			name="Spin to localize fallback",
			children=[counter(60, "Spin to localize counter"), Go("Spin to localize", linear=0, angular=-1)]
		)

		# move to chair
		b4 = pt.composites.Selector(
			name="move to pickup pose fallback",
			children=[GoTo("pick", with_aruco=False)])

		# lower head
		b5 = MoveHead("down")

		b6 = pt.composites.Selector(
			name="Looking for aruco cube fallback",
			children=[LookForAruco(), Go("Spin to find aruco", linear=0, angular=0.1)]
		)

		b7 = MoveArm("pick")

		b8 = pt.composites.Selector(
			name="move to place pose fallback",
			children=[CheckAruco(), GoTo("place", with_aruco=True)])

		b9 = MoveHead("down")

		b10 = MoveArm("place")

		behaviour_list = [b2, b3, b4, b5, b6, b7, b8, b9, b10]

		be = pt.composites.Selector(
			name="move to pickup pose fallback",
			children=[ResetTasks(), GoTo("pick", with_aruco=False)])

		inner_seq = pt.composites.Sequence(name="Inner sequence", children=[b2, b3, b4, b5, b6, b7, b8, b9, b10, be])

		# become the tree
		tree = RSequence(name="Main sequence", children=[b1, inner_seq])
		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(1)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)	

if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
