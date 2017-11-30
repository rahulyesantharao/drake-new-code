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

def normalize(v):
	mag = np.linalg.norm(v)
	if(mag==0):
		return v
	return v/mag

if __name__ == '__main__':
	# ********* NOTES *********
	# First we take the vertices of the region
	# Polytopes represented as A[2] (array of nx3 matrix), constraints are in b (nx1 matrix)
	# z[2]: sum(z)=1, zi = 0, 1: if z[i], then x is in A[i]
	# x is the decision variable, (3x1 matrix) represents the point in 3D space
	
	# ********* HARD CODE POINTS/REGIONS OF INTEREST *********
	# Set Goal Point
	dim = 3
	num_regions = 3
	# Create Polytope V-representations
	# P = []
	# v1 = np.array([(0,0,0), (0,0,1), (0,1,0), (1,0,0), (0,1,1), (1,0,1), (1,1,0), (0.5, 0.5, 0.5), (1,1,1)]) # numpy array of vertices of A1
	# print(v1)
	# print(ConvexHull(v1).vertices)
	# P.append(np.array([v1[i] for i in ConvexHull(v1).vertices])) # P[0] is the convex hull of the first polytope
	# print(P[0])
	# v2 = np.array([(1,1,1), (1,1,2), (1,2,1), (2,1,1), (1,2,2), (2,1,2), (2,2,1), (1.5, 1.5, 1.5), (2,2,2)]) # numpy array of vertices of A2
	# P.append(np.array([v2[i] for i in ConvexHull(v2).vertices])) # P[1] is the convex hull of the second polytope
	# print(P)

	# ********* CONVERT V-REPRESENTATIONS TO H-REPRESENTATIONS *********
	# Convert region convex hulls to H-representation
	# Basic Tetrahedron
	# basictet = [(0,0,0), (1,0,0), (0,1,0), (0,0,1)]
	chulls = []
	A = []
	b = []
	# R = np.array([[0, 1], [-1, 0]])
	for j in range(num_regions):
		# print("Region " + str(j) + ":")
		temp = np.zeros(dim)
		temp[0] += j
		pts = ConvexHull(np.random.rand(4, dim) + temp) # generate the vertices
		# pts = ConvexHull(np.array(basictet))
		print("TEST***")
		print(pts.simplices)
		chulls.append(pts)
		# P.append(np.array([pts.points[i] for i in pts.vertices]))
		A.append(np.delete(pts.equations, pts.equations.shape[1]-1, 1))
		b.append(-1*pts.equations[:,pts.equations.shape[1]-1])
		# #########
		# # print("Points:")
		# # for p in pts.points:
		# # 	print(p)
		# # print("Convex Hull:")
		# # for v in pts.vertices:
		# # 	print(pts.points[v])
		# chull = [pts.points[i] for i in pts.vertices] # create convex hull
		# chull.append(pts.points[pts.vertices[0]]) # add the first point (cc-order) again
		# P.append(np.array(chull)) # Add the convex hull to P (the list of V-reps)
		# A.append(np.zeros((P[j].shape[0]-1, P[j].shape[1])))
		# b.append(np.zeros(A[j].shape[0]))
		# # print("Sides:")
		# for i in range(A[j].shape[0]):
		# 	v = P[j][i+1] - P[j][i]
		# 	# print(v)
		# 	A[j][i] = np.matmul(R, v)
		# 	# print(A[j][i])
		# 	A[j][i] = normalize(A[j][i])
		# 	# print(A[j][i])
		# 	# print("")
		# 	b[j][i] = np.dot(A[j][i], P[j][i])

		# # TEST ##
		# # print("EQUATIONS:")
		# # print(pts.equations)
		# # print(type(pts.equations))
		# # print("A:")
		# # print(A[j])
		# # print("b:")
		# # print(b[j])

	x_lb = 0
	x_ub = num_regions + 2
	x_goal = np.array([1.5]*dim)
	x_goal[0] = num_regions/2+1

	# ********* SOLVE PROBLEM *********
	# Create optimization problem
	prog = mp.MathematicalProgram()

	# Create variables
	x = prog.NewContinuousVariables(dim, "x") # variable point
	for i in range(dim):
		prog.AddLinearConstraint(x_lb<=x[i]<=x_ub)

	z = prog.NewBinaryVariables(num_regions, "z") # Integer variables that represent the region the point will be in
	# Constrain z (TODO: Binary constraint on z)
	prog.AddLinearConstraint(np.sum(z) == 1) # only one is set
	
	# Create M (TODO: calculate this value)
	M = 100

	# Constrain the points to the regions
	for i in range(num_regions):
		for j in range(A[i].shape[0]):
			prog.AddLinearConstraint(A[i][j][0]*x[0]+A[i][j][1]*x[1]+A[i][j][2]*x[2] + M*z[i] <= b[i][j] + M)


	# Add objective
	prog.AddQuadraticCost((x[0]-x_goal[0])**2 + (x[1]-x_goal[1])**2 + (x[2]-x_goal[2])**2) # distance of x to the goal point

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
	ax = fig.add_subplot(111, projection='3d')
	plt.title("Minimize distance of point within " + str(num_regions) + " " + str(dim) + "-D Polytopes to Goal Point")

	# Plot regions
	for j in range(num_regions):
		print("Region " + str(j))
		for simplex in chulls[j].simplices:
			print(simplex)
			print(chulls[j].points[simplex])
			for i in range(len(simplex)):
				cur = simplex[i]
				n = simplex[(i+1)%(len(simplex))]
				# print([chulls[j].points[cur][0], chulls[j].points[n][0]])
				# print([chulls[j].points[cur][1], chulls[j].points[n][1]])
				# print([chulls[j].points[cur][2], chulls[j].points[n][2]])
				plt.plot([chulls[j].points[cur][0], chulls[j].points[n][0]], [chulls[j].points[cur][1], chulls[j].points[n][1]], [chulls[j].points[cur][2], chulls[j].points[n][2]], 'b')
			# plt.plot(chulls[j].points[simplex, 0], chulls[j].points[simplex, 1], chulls[j].points[simplex, 2], 'b')

	plt.plot([finalx[0]], [finalx[1]], [finalx[2]], 'g*', markersize=15, markerfacecolor='g') # solution
	ax.text(finalx[0], finalx[1], finalx[2], "SOL: (" + str(round(finalx[0], 3)) + ", " + str(round(finalx[1], 3)) + ", " + str(round(finalx[2], 3)) + ")")
	plt.plot([x_goal[0]], [x_goal[1]], [x_goal[2]], 'r*', markersize=15, markerfacecolor='r') # goal
	ax.text(x_goal[0], x_goal[1], x_goal[2], "GOAL: (" + str(round(x_goal[0], 3)) + ", " + str(round(x_goal[1], 3)) + ", " + str(round(x_goal[2], 3)) + ")")
	plt.show() # Show plot