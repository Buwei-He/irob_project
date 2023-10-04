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
			children=[counter(20, "Go back done?"), go("Go back!", linear=-0.2, angular=0)]
		)

		# tuck the arm
		b1 = tuckarm()

		# go to table
		b2 = pt.composites.Selector(
			name="Spin to localize fallback",
			children=[counter(58, "Spin to localize done?"), go("Spin to localize!", linear=0, angular=-1)]
		)

		# move to chair
		b3 = goto("pick", with_aruco=False)

		# lower head
		b4 = movehead("down")

		b5 = pt.composites.Selector(
			name="Looking for aruco cube fallback",
			children=[counter(100, "Aruco cube detected?"), lookforaruco(angular=-0.58)]
		)

		b6 = movearm("pick")

		b7 = movehead("up")

		b8 = goto("place", with_aruco=True)

		b9 = movehead("down")

		b10 = movearm("place")

		inner_seq = RSequence(name="Inner sequence", children=[b2, b3, b4, b5, b6, b7, b8, b9, b10])

		inner_loop = pt.composites.Selector(
			name="Pick and place fallback",
			children=[resetaruco(), inner_seq, lookforaruco(angular=0)]
		)

		# become the tree
		rospy.loginfo("Behaviour tree...")
		tree = RSequence(name="Main sequence", children=[b0, b1, inner_loop])
		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)	

if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
