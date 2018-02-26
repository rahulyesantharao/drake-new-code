from __future__ import absolute_import, division, print_function

from FootstepPlanner import FootstepPlanner
import lcm
import drc
import bot_core

import os

from pydrake.parsers import PackageMap
from pydrake import rbtree
from pydrake import getDrakePath

import math
import numpy as np

if __name__== "__main__":
	print("FootstepPlanner LCM server started.")
	lc = lcm.LCM()

	# Listen on LCM for footstep_request_t
	def request_handler(channel, data):
		request = drc.footstep_plan_request_t.decode(data)
		# ----- Send LCM message based on received info -----
		# Create Footstep planner and get solution
		f = FootstepPlanner(2)

		# Create Atlas object
		print("----------------------------------")
		pm = PackageMap()
		model = os.path.abspath(os.path.join(os.sep, "home", "rahuly", "Data", "drake-new", "drake-distro", "drake", "examples", "atlas", "urdf",
		                     "atlas_minimal_contact.urdf"))
		pm.PopulateUpstreamToDrake(model)
		robot = rbtree.RigidBodyTree(
			model, package_map=pm,
			floating_base_type=rbtree.FloatingBaseType.kQuaternion)
		t = request.initial_state.pose.translation
		r = request.initial_state.pose.rotation
		q = [t.x, t.y, t.z] + [r.w, r.x, r.y, r.z] + list(request.initial_state.joint_position)
		q = tuple(q)
		v = [0]*6 + list(request.initial_state.joint_velocity)
		v = tuple(v)
		kinsol = robot.doKinematics(q, v)
		r_startpos = robot.transformPoints(kinsol, np.zeros((3, 1)), robot.findFrame("r_foot_sole").get_frame_index(), 0)
		l_startpos = robot.transformPoints(kinsol, np.zeros((3, 1)), robot.findFrame("l_foot_sole").get_frame_index(), 0)
		
		print("r_startpos: " + str(r_startpos))
		print("l_startpos: " + str(l_startpos))
		print("----------------------------------")

		f.setStartRL(r_startpos, l_startpos)
		f.setGoal([request.goal_pos.translation.x, request.goal_pos.translation.y])
		f.setReachable([0,0], 0.5, [0,-0.6], 0.5)
		# f.setNominal(0.5)
		f.solveProgram()
		f.showSolution(save=False)
		# Package solution into footstep_plan_t
		plan = drc.footstep_plan_t()

		# - Footsteps
		plan.num_steps = 2*f.numFootsteps
		for fNum in range(0, f.numFootsteps):
			print("-----------" + str(fNum) + "-----------")
			# -- Right Footstep
			rstep = drc.footstep_t()
			rstep.pos.rotation.w = math.cos(f.footsteps[2*fNum][2]*0.5)
			rstep.pos.rotation.z = math.sin(f.footsteps[2*fNum][2]*0.5)
			rspos = np.array([[f.footsteps[2*fNum][0]], [f.footsteps[2*fNum][1]], [0]])
			# rs = np.array([f.footsteps[2*fNum][0], f.footsteps[2*fNum][1], 0])
			rs = robot.transformPoints(kinsol, rspos, robot.FindBody("r_foot").get_body_index(), robot.findFrame("r_foot_sole").get_frame_index())
			print("Right: " + str(rs))
			print("Right Angle: " + str((180/math.pi) * f.footsteps[2*fNum][2]))
			rstep.pos.translation.x = rs[0]
			rstep.pos.translation.y = rs[1]
			rstep.pos.translation.z = rs[2]
			# pos.rotation ??
			rstep.id = 2*fNum
			rstep.is_right_foot = True
			rstep.params = request.default_step_params
			plan.footsteps.append(rstep)
			# -- Left Footstep
			lstep = drc.footstep_t()
			lstep.pos.rotation.w = math.cos(f.footsteps[2*fNum+1][2]*0.5)
			lstep.pos.rotation.z = math.sin(f.footsteps[2*fNum+1][2]*0.5)
			lspos = np.array([[f.footsteps[2*fNum+1][0]], [f.footsteps[2*fNum+1][1]], [0]])
			# ls = np.array([f.footsteps[2*fNum+1][0], f.footsteps[2*fNum+1][1], 0])
			ls = robot.transformPoints(kinsol, lspos, robot.FindBody("l_foot").get_body_index(), robot.findFrame("l_foot_sole").get_frame_index())
			print("Left: "  + str(ls))
			print("Left Angle: " + str((180/math.pi) * f.footsteps[2*fNum+1][2]))
			lstep.pos.translation.x = ls[0]
			lstep.pos.translation.y = ls[1]
			lstep.pos.translation.z = ls[2]
			# pos.rotation ??
			lstep.id = 2*fNum+1
			lstep.is_right_foot = False
			lstep.params = request.default_step_params
			plan.footsteps.append(lstep)

		print("goal_pos: " + str(request.goal_pos.translation.x) + ", " + str(request.goal_pos.translation.y))

		# - IRIS Regions
		plan.num_iris_regions = request.num_iris_regions
		plan.iris_regions = request.iris_regions
		plan.iris_region_assignments = [-1]*plan.num_steps

		# - Params: AFTER WE HAVE REQUESTS WORKING
		plan.params = request.params

		# Send footstep_plan_t
		lc.publish("FOOTSTEP_PLAN", plan.encode())

	subscription = lc.subscribe("REQUEST", request_handler)

	try:
		while True:
			lc.handle()
	except KeyboardInterrupt:
		pass

	lc.unsubscribe(subscription)