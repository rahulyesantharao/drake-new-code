# Finds the best number of footsteps AND uses a convex region for the constrains on the footsteps
from __future__ import absolute_import

# Pydrake imports
import pydrake
import numpy as np
from pydrake.solvers import mathematicalprogram as mp
from pydrake.solvers.gurobi import GurobiSolver
import pydrake.symbolic as sym

# Plotter
import sys
import os
myDir = os.path.dirname(os.path.abspath(__file__))
parentDir = os.path.split(myDir)[0]
if(sys.path.__contains__(parentDir)):
    print('parent already in path')
    pass
else:
    print('parent directory added')
    sys.path.append(parentDir)
from FootstepPlanner import Plotter

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
	# linconst = np.array([-goal[0], -goal[1]])

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

	# For each set of left, right footsteps [1, numFootsteps-1], they must be within the reachable convex region defined by the previous step
	for fNum in range(1,numFootsteps):
		# ***** Make assumption that the orientation is roughly in the +x direction *****
		## FIGURE THIS OUT ##
		pass

	# TODO: Minimize distances of footsteps from each other
	for fNum in range(1,2*numFootsteps-1):
		# Add cost of consecutive footsteps
		prog.AddQuadraticCost((f[fNum][0]-f[fNum+1][0])**2 + (f[fNum][1]-f[fNum+1][1])**2)

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
	prog.AddQuadraticCost(2*((f[2*numFootsteps-1][0]-goal[0])**2 + (f[2*numFootsteps-1][1]-goal[1])**2))
	prog.AddQuadraticCost(2*((f[2*numFootsteps-2][0]-goal[0])**2 + (f[2*numFootsteps-2][1]-goal[1])**2))
	
	# Add cost (number of footsteps)
	prog.AddLinearCost(-np.sum(z) * 50)

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
	ansSteps = numFootsteps
	print(result)
	for fNum in range(numFootsteps):
		curZ = prog.GetSolution(z[fNum])
		if(int(curZ)==1 and ansSteps==numFootsteps): ansSteps = fNum+1
		print(str(fNum) + ": " + str(curZ))

	print("")

	test = Plotter(2, goal, np.array(ansFootsteps), ansSteps) # Add the convex reachable regions