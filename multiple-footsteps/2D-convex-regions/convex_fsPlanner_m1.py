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
from mpl_toolkits.mplot3d import Axes3D

# Convex Hull function
from scipy.spatial import ConvexHull

# Make numpy round printed values
np.set_printoptions(suppress=True)

# image-id generator
import sys
import os 
import string
import random
def id_gen(dim, size=6, chars=string.ascii_uppercase+string.digits):
	dir_path = os.path.dirname(os.path.realpath(__file__))
	results_dir = os.path.join(dir_path, 'images/')
	file_name = str(dim) + 'D-' + ''.join(random.choice(chars) for _ in range(size)) + '.png'
	if not os.path.isdir(results_dir):
		os.makedirs(results_dir)
	return results_dir + file_name

def normalize(v):
	mag = np.linalg.norm(v)
	if(mag==0):
		return v
	return v/mag


if __name__ == '__main__':
	# ********* SET DIMENSIONS, PARAMETERS *********
	# Set dimensions, number of regions
	dim = 2
	num_regions = 6
	# Footstep Constants
	XDISP = 1
	YDISP = 1
	MAXNUMFOOTSTEPS = 9
	FOOTDIST = 0.5


	# ********* CREATE CONVEX REACHABLE REGIONS *********
	# Create H-representations of random regions
	chulls = []
	A = []
	b = []
	for j in range(num_regions):
		# print("Region " + str(j) + ":")

		# Create random vertices and the convex hull
		temp = np.array([(j, FOOTDIST), (j, -FOOTDIST), (j+0.5, FOOTDIST), (j+0.5, -FOOTDIST)])
		pts = ConvexHull(temp) # generate the vertices and convex hull
		
		# print("TEST***")
		# print(pts.simplices)
		
		# Save full qhull object
		chulls.append(pts)

		# Extract the H-representation
		A.append(np.delete(pts.equations, pts.equations.shape[1]-1, 1))
		b.append(-1*pts.equations[:,pts.equations.shape[1]-1])

	# # Create random vertices and the convex hull
	# temp = np.array([(0.8, FOOTDIST+0.1), (1, 2*FOOTDIST), (5, FOOTDIST+0.1), (5, 2*FOOTDIST)])
	# pts = ConvexHull(temp) # generate the vertices and convex hull
	
	# # print("TEST***")
	# # print(pts.simplices)
	
	# # Save full qhull object
	# chulls.append(pts)

	# Extract the H-representation
	A.append(np.delete(pts.equations, pts.equations.shape[1]-1, 1))
	b.append(-1*pts.equations[:,pts.equations.shape[1]-1])	

	# Create bounds on x
	x_lb = 0
	x_ub = num_regions + 2

	# Create goal point
	x_goal = np.array([1]*dim) # Not inside any of the regions
	x_goal[0] = num_regions # In the middle of the random regions (x-dim)

	# num_regions+=1
	# ********* SOLVE PROBLEM *********
	for numFootsteps in range(1, MAXNUMFOOTSTEPS+1):
		# Create optimization problem
		prog = mp.MathematicalProgram()

		# Create list to hold footstep variables
		f = []

		# Create footstep variables
		for fNum in range(numFootsteps):
			fnL = prog.NewContinuousVariables(2, "fl"+str(fNum)) # Left Footstep
			fnR = prog.NewContinuousVariables(2, "fr"+str(fNum)) # Right Footstep
			f.append(fnL)
			f.append(fnR)

		# Starting points for left, right footsteps
		prog.AddLinearConstraint(f[0][0] == 0)
		prog.AddLinearConstraint(f[0][1] == FOOTDIST/2)
		prog.AddLinearConstraint(f[1][0] == 0)
		prog.AddLinearConstraint(f[1][1] == -FOOTDIST/2)


		# Constrain footsteps with distance from each other
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

		# Create list to hold binary variables
		H = []

		# Create binary variables
		for fNum in range(1, numFootsteps):
			hnL = prog.NewBinaryVariables(num_regions, "hl"+str(fNum)) # Left Footstep
			hnR = prog.NewBinaryVariables(num_regions, "hr"+str(fNum)) # Right Footstep

			# Constrain each footstep to exactly one convex region
			prog.AddLinearConstraint(np.sum(hnL) == 1) # only one is set
			prog.AddLinearConstraint(np.sum(hnR) == 1) # only one is set

			H.append(hnL)
			H.append(hnR)

		# Create M (TODO: calculate this value)
		M = 100

		# Constrain the footsteps to the regions
		for fNum in range(1, numFootsteps-1):
			for i in range(num_regions):
				for j in range(A[i].shape[0]):
					prog.AddLinearConstraint(A[i][j][0]*f[2*fNum][0]+A[i][j][1]*f[2*fNum][1] + M*H[2*fNum][i] <= b[i][j] + M) # Left footstep constraint
					prog.AddLinearConstraint(A[i][j][0]*f[2*fNum+1][0]+A[i][j][1]*f[2*fNum+1][1] + M*H[2*fNum+1][i] <= b[i][j] + M) # Right footstep constraint

		# Add cost (distance of final footsteps to goal)
		prog.AddQuadraticCost((f[2*numFootsteps-2][0]-x_goal[0])**2 + (f[2*numFootsteps-2][1]-x_goal[1])**2)
		prog.AddQuadraticCost((f[2*numFootsteps-1][0]-x_goal[0])**2 + (f[2*numFootsteps-1][1]-x_goal[1])**2)

		# Solve the problem
		solver = GurobiSolver()
		assert(solver.available())
		assert(solver.solver_type()==mp.SolverType.kGurobi)
		result = solver.Solve(prog)
		assert(result == mp.SolutionResult.kSolutionFound)
		print("Goal: " + str(x_goal))
		# Print out the footsteps
		print("Case " + str(numFootsteps) + ":")
		print(result)
		for fNum in range(2*numFootsteps):
			print(prog.GetSolution(f[fNum]))
		print("")

		# Get solution points
		finalRstep = prog.GetSolution(f[2*numFootsteps-1])
		finalLstep = prog.GetSolution(f[2*numFootsteps-2])
		finalPos = np.array([(finalLstep[0]+finalRstep[0])/2.0, (finalLstep[1]+finalRstep[1])/2.0])


		# ********* GRAPH PROBLEM *********
		# Create figure
		fig = plt.figure(1, (20, 10))
		sb = fig.add_subplot(111)
		if(np.allclose(x_goal, finalPos)):
			plt.title("SOLUTION " + str(numFootsteps) + " L/R footsteps within convex bounded regions to reach a goal point")
		else:
			plt.title(str(numFootsteps) + " L/R footsteps within convex bounded regions to reach a goal point")
		plt.xlabel("X Displacement")
		plt.ylabel("Y Displacement")

		# Plot regions
		for j in range(num_regions):
			print("Region " + str(j))
			for simplex in chulls[j].simplices:
				print(simplex)
				print(chulls[j].points[simplex])
				plt.plot(chulls[j].points[simplex, 0], chulls[j].points[simplex, 1], 'b')

		# Plot goal
		plt.plot([x_goal[0]], [x_goal[1]], 'r*', markersize=15, markerfacecolor='r') # goal
		plt.annotate("GOAL: (" + str(round(x_goal[0], 3)) + ", " + str(round(x_goal[1], 3)) + ")", xy=(x_goal[0], x_goal[1]))

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

		plt.show()

		if(np.allclose(finalPos, x_goal)):
			break