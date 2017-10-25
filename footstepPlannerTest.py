from __future__ import absolute_import

# import sys

import pydrake
import numpy as np
from pydrake.solvers import mathematicalprogram as mp
import pydrake.symbolic as sym

XDISP = 5
YDISP = 5
MAXNUMFOOTSTEPS = 10
np.set_printoptions(suppress=True)

if __name__ == '__main__':
	goalX, goalY = input("Goal Displacement (X,Y): ")
	goal = np.array([goalX, goalY])
	linconst = np.array([-goal[0], -goal[1]])
	
	ans = -1
	ansfootsteps = []

	# try reaching goal in numFootsteps
	for numFootsteps in (range(1,MAXNUMFOOTSTEPS+1)):
		
		# Create quadratic program
		prog = mp.MathematicalProgram()
		
		# Create list to hold footstep variables
		f = []

		# Create footstep variables
		for fNum in range(numFootsteps):
			fn = prog.NewContinuousVariables(2, "f"+str(fNum))
			f.append(fn)

		# Add displacement constraints (dist of footsteps fNum, fNum-1: go from 1 to numFootsteps-1)
		prog.AddLinearConstraint(f[0][0] <= XDISP)
		prog.AddLinearConstraint(f[0][0] >= -XDISP)
		prog.AddLinearConstraint(f[0][1] <= YDISP)
		prog.AddLinearConstraint(f[0][1] >= -YDISP)
		for fNum in range(1,numFootsteps):
			prog.AddLinearConstraint(f[fNum][0]-f[fNum-1][0] <= XDISP)
			prog.AddLinearConstraint(f[fNum][0]-f[fNum-1][0] >= -XDISP)
			prog.AddLinearConstraint(f[fNum][1]-f[fNum-1][1] <= YDISP)
			prog.AddLinearConstraint(f[fNum][1]-f[fNum-1][1] >= -YDISP)


		# Add costs: minimize the distances of each footstep from one another
		# HOW TO MAKE THESE LESS IMPORTANT/SECONDARY TO THE OVERALL QUADRATIC COST?
		# prog.AddQuadraticCost(np.eye(2), np.zeros(2), f[0])
		for fNum in range(1,numFootsteps):
			pass

		# Add cost (distance of final footstep to goal)
		prog.AddQuadraticCost(np.eye(2), linconst, f[numFootsteps-1])
		
		# Solve the program
		result = prog.Solve()
		finalstep = prog.GetSolution(f[numFootsteps-1])

		print("Case " + str(numFootsteps) + ":")
		print(result)
		for fNum in range(numFootsteps):
			print(prog.GetSolution(f[fNum]))
		print("")

		# Make sure it can be solved
		assert(result==mp.SolutionResult.kSolutionFound)

		# If the solution is good, save it
		if(np.allclose(finalstep, goal)):
			ans = numFootsteps
			for fNum in range(numFootsteps):
				ansfootsteps.append(prog.GetSolution(f[fNum]))
			break

	print("*** FINAL ANSWER ***")
	print(ans)
	for footstep in ansfootsteps:
		print(footstep)