import lcm
import drc
import bot_core
from Plotter import Plotter 

import numpy as np
from scipy.spatial import ConvexHull

if __name__== "__main__":
	lc = lcm.LCM()

	# CONSTANTS FOR TEST
	START_POS = [0,0]
	GOAL_POS = np.array([1,1])
	REACHABLE = ConvexHull([(0,-0.2), (0.5,0), (0,0.2), (-0.3,0)])
	Y_OFFSET = 0.3

	# Listener for response with footstep_plan_t
	def plan_handle(channel, data):
		plan = drc.footstep_plan_t.decode(data)
		# -- PLOT FOOTSTEP PLAN --
		print("-------------------------------------------")
		footsteps = [] # Unpack footstps into list
		for f in plan.footsteps:
			footsteps.append([f.pos.translation.x, f.pos.translation.y])
		footsteps = np.array(footsteps)
		p = Plotter(2, GOAL_POS, footsteps, plan.num_steps/2, REACHABLE, Y_OFFSET)
		p.plot()
		print("-------------------------------------------")
		p.show()

	# Subscribe listener
	subscription = lc.subscribe("FOOTSTEP_PLAN", plan_handle)

	# Publish footstep_plan_request_t 
	request = drc.footstep_plan_request_t()
	
	goalpos = bot_core.position_3d_t()
	goalpos.translation.x = GOAL_POS[0]
	goalpos.translation.y = GOAL_POS[1]
	request.goal_pos = goalpos

	startstate = bot_core.robot_state_t()
	startstate.pose.translation.x = START_POS[0]
	startstate.pose.translation.y = START_POS[1]
	request.initial_state = startstate

	lc.publish("REQUEST", request.encode())

	# Listen for response with footstep_plan_t
	try:
		while True:
			lc.handle()
	except KeyboardInterrupt:
		pass

	lc.unsubscribe(subscription)