from __future__ import absolute_import

# Pydrake imports
import pydrake
import numpy as np
from pydrake.solvers import mathematicalprogram as mp
from pydrake.solvers.gurobi import GurobiSolver
import pydrake.symbolic as sym

# Convex Hull class
from scipy.spatial import ConvexHull

# Plotter
from Plotter import Plotter

# Make numpy round printed values
np.set_printoptions(suppress=True)

class FootstepPlanner:
	MAXFOOTSTEPS = 10
	STARTL = (0, 1)
	STARTR = (0, -1)
	def __init__(self, dim):
		assert(dim==2), "must be 2D; it is: " + str(dim)
		self.dim = dim
		# Footstep plan
		self.footsteps = None
		self.goal = None
		self.numFootsteps = -1
		self.upToDate = True
		# Convex reachable regions based on previous footstep
		self.reachableChull = None
		self.reachableA = None
		self.reachableb = None
		self.hasNominal = False
		self.nominal = -1
		self.yOffset = 0
		# Obstacle Free Regions
		self.hasObstacleFree = False
		self.num_regions = -1
		self.oChulls = None
		self.oA = None
		self.ob = None

	# pointSet needs to be centered around the origin	
	def setReachable(self, pointSet, yOffset):
		assert(isinstance(pointSet, list)), "pointSet needs to be a list of points; instead it is " + str(type(pointSet))
		assert(isinstance(yOffset, float)), "yOffset needs to be a float; instead it is " + str(type(yOffset))
		self.reachableChull = ConvexHull(np.array(pointSet))
		self.reachableA = np.delete(self.reachableChull.equations, self.reachableChull.equations.shape[1]-1, 1)
		self.reachableb = -1*self.reachableChull.equations[:,self.reachableChull.equations.shape[1]-1]
		self.yOffset = yOffset
		self.upToDate = False

	def setNominal(self, nominalRatio):
		assert(isinstance(nominalRatio, float) and (0<nominalRatio<1)), "nominalRatio needs to be in (0,1): " + str(nominalRatio)
		self.nominal = nominalRatio
		self.hasNominal = True
		self.upToDate = False

	def setGoal(self, goal):
		assert(isinstance(goal, list) and len(goal)==self.dim), "goal needs to be a " + str(self.dim) + "-length list of coordinates; it is " + str(goal)
		self.goal = np.array(goal)
		self.upToDate = False

	def setObstacleFree(self, regions):
		assert(isinstance(regions, list)), "regions needs to be a list of regions (lists of points): " + str(type(regions))
		self.num_regions = 0
		self.oChulls = []
		self.oA = []
		self.ob = []
		for region in regions:
			chull = ConvexHull(np.array(region))
			self.oChulls.append(chull)
			self.oA.append(np.delete(chull.equations, chull.equations.shape[1]-1, 1))
			self.ob.append(-1*chull.equations[:,chull.equations.shape[1]-1])
			self.num_regions += 1
		self.hasObstacleFree = True
		self.upToDate = False

	def solveProgram(self):
		if(not self.upToDate):
			prog = mp.MathematicalProgram()
			M = 100
			
			# SET UP FOOTSTEP VARIABLES
			f = []
			for fNum in range(FootstepPlanner.MAXFOOTSTEPS):
				fnR = prog.NewContinuousVariables(self.dim, "fr" + str(fNum)) # Right Footstep
				fnL = prog.NewContinuousVariables(self.dim, "fl" + str(fNum)) # Left Footstep
				f.append(fnR)
				f.append(fnL)
			n = prog.NewBinaryVariables(2*FootstepPlanner.MAXFOOTSTEPS, "n") # binary variables for nominal regions

			# CONSTRAIN WITH REACHABLE REGIONS
			# Start position
			prog.AddLinearConstraint(f[0][0] == FootstepPlanner.STARTR[0])
			prog.AddLinearConstraint(f[0][1] == FootstepPlanner.STARTR[1])
			prog.AddLinearConstraint(f[1][0] == FootstepPlanner.STARTL[0])
			prog.AddLinearConstraint(f[1][1] == FootstepPlanner.STARTL[1])
			# All other footsteps
			for fNum in range(1, FootstepPlanner.MAXFOOTSTEPS):
				for i in range(self.reachableA.shape[0]):
					# Constrain footsteps
					prog.AddLinearConstraint(self.reachableA[i][0]*(f[2*fNum][0]-f[2*fNum-1][0]) + self.reachableA[i][1]*(f[2*fNum][1]-(f[2*fNum-1][1]-self.yOffset)) <= self.reachableb[i]) # Right Footstep (2*fNum) to previous left (2*fNum-1)
					prog.AddLinearConstraint(self.reachableA[i][0]*(f[2*fNum+1][0]-f[2*fNum][0]) + self.reachableA[i][1]*(f[2*fNum+1][1]-(f[2*fNum][1]+self.yOffset)) <= self.reachableb[i]) # Left Footstep (2*fNum+1) to previous right (2*fNum)
					if(self.hasNominal): # Nominal Regions
						prog.AddLinearConstraint(self.reachableA[i][0]*(f[2*fNum][0]-f[2*fNum-1][0]) + self.reachableA[i][1]*(f[2*fNum][1]-(f[2*fNum-1][1]-self.yOffset)) + n[2*fNum]*M <= self.nominal*self.reachableb[i] + M) # Right Footstep (2*fNum) to previous left (2*fNum-1)
						prog.AddLinearConstraint(self.reachableA[i][0]*(f[2*fNum+1][0]-f[2*fNum][0]) + self.reachableA[i][1]*(f[2*fNum+1][1]-(f[2*fNum][1]+self.yOffset)) + n[2*fNum+1]*M <= self.nominal*self.reachableb[i] + M) # Left Footstep (2*fNum+1) to previous right (2*fNum)

			# CONSTRAIN TO OBSTACLE FREE REGIONS
			if(self.hasObstacleFree):
				H = []
				for fNum in range(1, FootstepPlanner.MAXFOOTSTEPS):
					hnR = prog.NewBinaryVariables(self.num_regions, "hr"+str(fNum)) # Right Footstep
					hnL = prog.NewBinaryVariables(self.num_regions, "hl"+str(fNum)) # Left Footstep

					# Constrain each footstep to exactly one convex region
					prog.AddLinearConstraint(np.sum(hnR) == 1) # only one is set
					prog.AddLinearConstraint(np.sum(hnL) == 1) # only one is set

					H.append(hnR)
					H.append(hnL)
				# Constrain the footsteps to the regions
				for fNum in range(1, FootstepPlanner.MAXFOOTSTEPS-1):
					for i in range(self.num_regions):
						for j in range(self.oA[i].shape[0]):
							prog.AddLinearConstraint(self.oA[i][j][0]*f[2*fNum][0]+self.oA[i][j][1]*f[2*fNum][1] + M*H[2*fNum][i] <= self.ob[i][j] + M) # Right footstep constraint
							prog.AddLinearConstraint(self.oA[i][j][0]*f[2*fNum+1][0]+self.oA[i][j][1]*f[2*fNum+1][1] + M*H[2*fNum+1][i] <= self.ob[i][j] + M) # Left footstep constraint

			# OPTIMAL NUMBER OF FOOTSTEPS
			z = prog.NewBinaryVariables(FootstepPlanner.MAXFOOTSTEPS, "z")
			for fNum in range(FootstepPlanner.MAXFOOTSTEPS-1):
				prog.AddLinearConstraint(z[fNum]<=z[fNum+1])
			prog.AddLinearConstraint(z[FootstepPlanner.MAXFOOTSTEPS-1]-z[0]==1)

			for fNum in range(FootstepPlanner.MAXFOOTSTEPS): # if z[i], then the ith footstep is the same as the final footstep
				prog.AddLinearConstraint(f[2*fNum][0] + M*z[fNum] <= f[2*FootstepPlanner.MAXFOOTSTEPS-2][0]+M)
				prog.AddLinearConstraint(f[2*fNum][1] + M*z[fNum] <= f[2*FootstepPlanner.MAXFOOTSTEPS-2][1]+M)
				prog.AddLinearConstraint(-f[2*fNum][0] + M*z[fNum] <= -f[2*FootstepPlanner.MAXFOOTSTEPS-2][0]+M)
				prog.AddLinearConstraint(-f[2*fNum][1] + M*z[fNum] <= -f[2*FootstepPlanner.MAXFOOTSTEPS-2][1]+M)
				prog.AddLinearConstraint(f[2*fNum+1][0] + M*z[fNum] <= f[2*FootstepPlanner.MAXFOOTSTEPS-1][0]+M)
				prog.AddLinearConstraint(f[2*fNum+1][1] + M*z[fNum] <= f[2*FootstepPlanner.MAXFOOTSTEPS-1][1]+M)
				prog.AddLinearConstraint(-f[2*fNum+1][0] + M*z[fNum] <= -f[2*FootstepPlanner.MAXFOOTSTEPS-1][0]+M)
				prog.AddLinearConstraint(-f[2*fNum+1][1] + M*z[fNum] <= -f[2*FootstepPlanner.MAXFOOTSTEPS-1][1]+M)


			# ADD COSTS
			# Cost of consecutive footsteps (with nominal regions considered)
			for fNum in range(1,2*FootstepPlanner.MAXFOOTSTEPS-1):
				prog.AddQuadraticCost(((f[fNum][0]-f[fNum+1][0])**2 + (f[fNum][1]-f[fNum+1][1])**2) + 2*(1-n[fNum]))

			# Cost of number of footsteps
			prog.AddLinearCost(-np.sum(z) * 5)

			# Cost of distance of final position to goal
			prog.AddQuadraticCost(1*((f[2*FootstepPlanner.MAXFOOTSTEPS-1][0]-self.goal[0])**2 + (f[2*FootstepPlanner.MAXFOOTSTEPS-1][1]-self.goal[1])**2))
			prog.AddQuadraticCost(1*((f[2*FootstepPlanner.MAXFOOTSTEPS-2][0]-self.goal[0])**2 + (f[2*FootstepPlanner.MAXFOOTSTEPS-2][1]-self.goal[1])**2))

			# SOLVE PROBLEM
			solver = GurobiSolver()
			assert(solver.available())
			assert(solver.solver_type()==mp.SolverType.kGurobi)
			result = solver.Solve(prog)
			assert(result == mp.SolutionResult.kSolutionFound)

			# SAVE SOLUTION
			self.footsteps = []
			self.numFootsteps = 0
			for fNum in range(FootstepPlanner.MAXFOOTSTEPS):
				self.numFootsteps += 1
				self.footsteps.append(prog.GetSolution(f[2*fNum]))
				self.footsteps.append(prog.GetSolution(f[2*fNum+1]))
				if(prog.GetSolution(z[fNum]) == 1):
					break
			self.footsteps = np.array(self.footsteps)
			self.upToDate = True

	def showSolution(self, save=False):
		print(str(self.numFootsteps) + " Footsteps:")
		for f in self.footsteps:
			print(f)

		if(not self.upToDate):
			print("Solution not up to date; please run solve() to update the solution")
		p = Plotter(self.dim, self.goal, self.footsteps, self.numFootsteps, self.reachableChull, self.yOffset)
		if(self.hasNominal):
			p.setNominal(self.nominal)
		if(self.hasObstacleFree):
			p.setObstacleFree(self.oChulls)
		p.plot()
		if(save):
			p.save()
		p.show()
		