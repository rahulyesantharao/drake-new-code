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
	num_regions = 5
	
	# ********* CREATE REGIONS OF INTEREST *********
	# Create H-representations of random regions
	chulls = []
	A = []
	b = []
	for j in range(num_regions):
		# print("Region " + str(j) + ":")

		# Create random vertices and the convex hull
		temp = np.zeros(dim)
		temp[0] += j # offset for regions
		pts = ConvexHull(np.random.rand(4, dim) + temp) # generate the vertices and convex hull
		
		# print("TEST***")
		# print(pts.simplices)
		
		# Save full qhull object
		chulls.append(pts)

		# Extract the H-representation
		A.append(np.delete(pts.equations, pts.equations.shape[1]-1, 1))
		b.append(-1*pts.equations[:,pts.equations.shape[1]-1])

	# Create bounds on x
	x_lb = 0
	x_ub = num_regions + 2

	# Create goal point
	x_goal = np.array([1.5]*dim) # Not inside any of the regions
	x_goal[0] = num_regions/2+1 # In the middle of the random regions (x-dim)

	# ********* SOLVE PROBLEM *********
	# Create optimization problem
	prog = mp.MathematicalProgram()

	# Create variables
	x = prog.NewContinuousVariables(dim, "x") # variable point
	for i in range(dim):
		prog.AddLinearConstraint(x_lb<=x[i]<=x_ub)

	z = prog.NewBinaryVariables(num_regions, "z") # Integer variables that represent the region the point will be in
	prog.AddLinearConstraint(np.sum(z) == 1) # only one is set
	
	# Create M (TODO: calculate this value)
	M = 100

	# Constrain the points to the regions
	for i in range(num_regions):
		for j in range(A[i].shape[0]):
			prog.AddLinearConstraint(A[i][j][0]*x[0]+A[i][j][1]*x[1] + M*z[i] <= b[i][j] + M)

	# Add objective
	prog.AddQuadraticCost((x[0]-x_goal[0])**2 + (x[1]-x_goal[1])**2) # distance of x to the goal point

	# Solve problem
	solver = GurobiSolver()
	assert(solver.available())
	assert(solver.solver_type()==mp.SolverType.kGurobi)
	result = solver.Solve(prog)
	assert(result == mp.SolutionResult.kSolutionFound)
	print("Goal: " + str(x_goal))
	finalx = prog.GetSolution(x)
	print("Final Solution: " + str(finalx))

	# ********* GRAPH PROBLEM *********
	# Create figure
	fig = plt.figure(1, (20, 10))
	plt.title("Minimize distance of point within " + str(num_regions) + " " + str(dim) + "-D Polytopes to Goal Point")

	# Plot regions
	for j in range(num_regions):
		print("Region " + str(j))
		for simplex in chulls[j].simplices:
			print(simplex)
			print(chulls[j].points[simplex])
			plt.plot(chulls[j].points[simplex, 0], chulls[j].points[simplex, 1], 'b')

	plt.plot([finalx[0]], [finalx[1]], 'g*', markersize=15, markerfacecolor='g') # solution
	plt.annotate("SOL: (" + str(round(finalx[0], 3)) + ", " + str(round(finalx[1], 3)) + ")", xy=(finalx[0], finalx[1]))
	plt.plot([x_goal[0]], [x_goal[1]], 'r*', markersize=15, markerfacecolor='r') # goal
	plt.annotate("GOAL: (" + str(round(x_goal[0], 3)) + ", " + str(round(x_goal[1], 3)) + ")", xy=(x_goal[0], x_goal[1]))

	if(len(sys.argv)>1 and sys.argv[1]=='-s'): # save image
		fig.savefig(id_gen(dim))
	plt.show() # Show plot