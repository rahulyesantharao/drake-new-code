from __future__ import absolute_import

# Pydrake imports
import pydrake
import numpy as np
from pydrake.solvers import mathematicalprogram as mp
from pydrake.solvers.gurobi import GurobiSolver
import pydrake.symbolic as sym

# Pyplot to plot footsteps
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Make numpy round printed values
np.set_printoptions(suppress=True)

if __name__ == '__main__':
	
	# Constants (temporary)
	XDISP = 5
	YDISP = 2
	FOOTDIST = 2
	numFootsteps = 9

	# Set up graph figure
	# fig = plt.figure(1, (20, 10))

	# Set up goal point
	goalX, goalY = 10, 10 #input("Goal Displacement (X,Y): ")
	goal = np.array([goalX, goalY])
	linconst = np.array([-goal[0], -goal[1]])

	# Try reaching the goal in numFootsteps
	# Create quadratic program
	prog = mp.MathematicalProgram()
	
	# Create list to hold footstep variables
	f = []

	# Create footstep variables
	for fNum in range(numFootsteps):
		fnL = prog.NewContinuousVariables(2, "fl"+str(fNum)) # Left Footstep
		fnR = prog.NewContinuousVariables(2, "fr"+str(fNum)) # Right Footstep
		f.append(fnL)
		f.append(fnR)

	# Add displacement constraints

	# Starting points for left, right footsteps
	prog.AddLinearConstraint(f[0][0] == 0)
	prog.AddLinearConstraint(f[0][1] == FOOTDIST/2)
	prog.AddLinearConstraint(f[1][0] == 0)
	prog.AddLinearConstraint(f[1][1] == -FOOTDIST/2)

	# For each set of left, right footsteps [1, numFootsteps-1], they must be the same distance from the previous left/right footstep
	for fNum in range(1,numFootsteps):
		# ***** Make assumption that the orientation is roughly in the +x direction *****
		# Current Left Footstep (2*fNum) is positioned based on previous Right Footstep (2*fNum-1)
		prog.AddLinearConstraint(f[2*fNum][1]>=f[2*fNum-1][1]+YDISP*0.5) # More than 0.5*FOOTDIST above
		prog.AddLinearConstraint(f[2*fNum][1]<=f[2*fNum-1][1]+YDISP*1.5) # Less than 1.5*FOOTDIST below
		prog.AddLinearConstraint(f[2*fNum][0]>=f[2*fNum-1][0]-XDISP)
		prog.AddLinearConstraint(f[2*fNum][0]<=f[2*fNum-1][0]+XDISP)

		# Current Right Footstep (2*fNum+1) is positioned based on previous Left Footstep (2*fNum)
		prog.AddLinearConstraint(f[2*fNum+1][1]<=f[2*fNum][1]-YDISP*0.5) # More than 0.5*FOOTDIST below
		prog.AddLinearConstraint(f[2*fNum+1][1]>=f[2*fNum][1]-YDISP*1.5) # Less than 1.5*FOOTDIST above
		prog.AddLinearConstraint(f[2*fNum+1][0]>=f[2*fNum][0]-XDISP)
		prog.AddLinearConstraint(f[2*fNum+1][0]<=f[2*fNum][0])

	# TODO: Minimize distances of footsteps from each other
	for fNum in range(1,numFootsteps):
		pass
		# Left to left
		# prog.AddQuadraticCost()

	# big M
	M = 100

	z = prog.NewBinaryVariables(numFootsteps, "z")
	for fNum in range(numFootsteps-1):
		prog.AddLinearConstraint(z[fNum]<=z[fNum+1])
	prog.AddLinearConstraint(z[numFootsteps-1]-z[0]==1)
	
	for fNum in range(numFootsteps):
		pass
		# z[i] -> f[i]<=f[n] && f[i]>=f[n]
		prog.AddLinearConstraint(f[2*fNum][0] + M*z[fNum] <= f[2*numFootsteps-2][0]+M)
		prog.AddLinearConstraint(f[2*fNum][1] + M*z[fNum] <= f[2*numFootsteps-2][1]+M)
		prog.AddLinearConstraint(-f[2*fNum][0] + M*z[fNum] <= -f[2*numFootsteps-2][0]+M)
		prog.AddLinearConstraint(-f[2*fNum][1] + M*z[fNum] <= -f[2*numFootsteps-2][1]+M)
		prog.AddLinearConstraint(f[2*fNum+1][0] + M*z[fNum] <= f[2*numFootsteps-1][0]+M)
		prog.AddLinearConstraint(f[2*fNum+1][1] + M*z[fNum] <= f[2*numFootsteps-1][1]+M)
		prog.AddLinearConstraint(-f[2*fNum+1][0] + M*z[fNum] <= -f[2*numFootsteps-1][0]+M)
		prog.AddLinearConstraint(-f[2*fNum+1][1] + M*z[fNum] <= -f[2*numFootsteps-1][1]+M)

	# Add cost (distance of final footsteps to goal)
	prog.AddQuadraticCost(10*np.eye(2), 10*linconst, f[2*numFootsteps-1])
	prog.AddQuadraticCost(10*np.eye(2), 10*linconst, f[2*numFootsteps-2])
	
	# Add cost (number of footsteps)
	prog.AddLinearCost(-np.sum(z) * 0.5)

	# Solve the program
	solver = GurobiSolver()
	assert(solver.available())
	assert(solver.solver_type()==mp.SolverType.kGurobi)
	result = solver.Solve(prog)
	assert(result == mp.SolutionResult.kSolutionFound)
	finalRstep = prog.GetSolution(f[2*numFootsteps-1])
	finalLstep = prog.GetSolution(f[2*numFootsteps-2])
	finalPos = np.array([(finalLstep[0]+finalRstep[0])/2.0, (finalLstep[1]+finalRstep[1])/2.0])
	# Print out the z
	print(result)
	for fNum in range(numFootsteps):
		print(str(fNum) + ": " + str(prog.GetSolution(z[fNum])))
	print("")

	# Make sure it can be solved
	assert(result==mp.SolutionResult.kSolutionFound)

	fig = plt.figure(1, (20, 10))
	sb = fig.add_subplot(111)
	
	# Left footsteps
	for l in range(numFootsteps):
		pt = prog.GetSolution(f[2*l])
		plt.plot(pt[0], pt[1], 'ro')
		lbl = "L"+str(l)
		h = ""
		if(l%2): h="|"
		plt.annotate(lbl + "(" + str(round(pt[0], 3)) + ", " + str(round(pt[1], 3)) + ")", xy=(pt[0], pt[1]))
		sb.add_patch(
			patches.Rectangle(
				(pt[0]-XDISP, pt[1]-1.5*YDISP),
				XDISP,
				YDISP,
				hatch=h,
				facecolor='red',
				edgecolor='red',
				label=lbl,
				alpha=0.1
			)
		)
	# Right footsteps
	for r in range(numFootsteps):
		pt = prog.GetSolution(f[2*r+1])
		plt.plot(pt[0], pt[1], 'go')
		lbl = "R"+str(r)
		h = ""
		if(r%2): h="|"
		plt.annotate(lbl + "(" + str(round(pt[0], 3)) + ", " + str(round(pt[1], 3)) + ")", xy=(pt[0], pt[1]))
		sb.add_patch(
			patches.Rectangle(
				(pt[0]-XDISP, pt[1]+0.5*YDISP),
				2*XDISP,
				YDISP,
				hatch=h,
				facecolor='green',
				edgecolor='green',
				label=lbl,
				alpha=0.1
			)
		)
	
	# Plot goal point
	plt.plot(goal[0], goal[1], 'bo')
	plt.annotate("GOAL: (" + str(round(goal[0], 3)) + ", " + str(round(goal[1], 3)) + ")", xy=(goal[0], goal[1]))
	
	# Format
	plt.xlabel("X Displacement")
	plt.ylabel("Y Displacement")
	plt.title(str(numFootsteps) + " Footsteps")

	# mng = plt.get_current_fig_manager()
	# mng.window.showMaximized()
	plt.show()