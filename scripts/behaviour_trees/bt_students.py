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
			children=[counter(20, "Done?"), go("Go back!", linear=-0.2, angular=0)]
		)

		# tuck the arm
		b1 = tuckarm()

		# go to table
		b2 = pt.composites.Selector(
			name="Spin to localize fallback",
			children=[counter(58, "Done?"), go("Spin to localize!", linear=0, angular=-1)]
		)

		# move to chair
		b3 = goto("pick")

		# lower head
		b4 = movehead("down")

		b5 = detectaruco()

		b6 = movearm("pick")

		b7 = goto("place")

		b8 = movearm("place")

		# become the tree
		tree = RSequence(name="Main sequence", children=[b0, b1, b2, b3, b4, b5, b6, b7, b8])
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
