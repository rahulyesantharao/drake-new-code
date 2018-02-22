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

import math

# Make numpy round printed values
np.set_printoptions(suppress=True)

class FootstepPlanner:
	MAXFOOTSTEPS = 8
	NUM_LINEAR_PIECES = 5

	# SINE PIECES: [0, 1) U [1, pi-1) U [pi-1, pi+1) U [pi+1, 2pi-1) U [2pi-1, 2pi)
	# linear approximation of sine
	PS = [0, 1, math.pi-1, math.pi+1, 2*math.pi-1, 2*math.pi]
	GS = [1, 0, -1, 0, 1]
	HS = [0, 1, math.pi, -1, -2*math.pi]

	# COSINE PIECES: [0, pi/2-1) U [pi/2-1, pi/2+1) U [pi/2+1, 3pi/2-1) U [3pi/2-1, 3pi/2+1) U [3pi/2+1, 2pi)
	# linear approximation of cosine
	PC = [0, math.pi/2 - 1, math.pi/2 + 1, 3*math.pi/2 - 1, 3*math.pi/2 + 1, 2*math.pi]
	GC = [0, -1, 0, 1, 0]
	HC = [1, math.pi/2, -1, -3*math.pi/2, 1]

	def __init__(self, dim):
		assert(dim==2), "must be 2D; it is: " + str(dim)
		self.dim = dim
		# Footstep plan
		self.footsteps = None
		self.cosApprox = None
		self.sinApprox = None
		self.goal = None
		self.start = None
		self.startL = None
		self.startR = None
		self.numFootsteps = -1
		self.upToDate = True
		# Convex reachable regions based on previous footstep
		self.c1 = None
		self.c2 = None
		self.r1 = -1
		self.r2 = -1
		self.hasNominal = False
		self.nominal = -1
		# Obstacle Free Regions
		self.hasObstacleFree = False
		self.num_regions = -1
		self.oChulls = None
		self.oA = None
		self.ob = None

	# The reachable region is defined by two circles, given by their centers and radii
	def setReachable(self, center1, radius1, center2, radius2):
		assert(isinstance(center1, list) and len(center1)==self.dim and isinstance(center2, list) and len(center2)==self.dim), "center1, center2 must be " + str(self.dim) + "D lists of coordinates; instead they are " + str(center1) + ", " + str(center2)
		assert(isinstance(radius1, float) and isinstance(radius2, float)), "radius1, radius2 need to be floats; instead they are " + str(type(radius1)) + ", " + str(type(radius2))
		self.c1 = center1
		self.r1 = radius1
		self.c2 = center2
		self.r2 = radius2
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

	def setStart(self, start):
		assert(isinstance(start, list) and len(start)==self.dim), "start needs to be a " + str(self.dim) + "-length list of coordinates; it is " + str(start)
		self.start = np.array(start)
		self.startL = (self.start[0], self.start[1]+0.12)
		self.startR = (self.start[0], self.start[1]-0.12)
		self.upToDate = False

	def setStartRL(self, startR, startL):
		# assert(isinstance(startL, list) and isinstance(startR, list) and len(startL)==self.dim and len(startR)==self.dim), "startL, startR need to be " + str(self.dim) + "-length tuples of coordinates; they are: " + type(startL) + ", " + type(startR)
		self.startL = (startL[0][0], startL[1][0])
		self.startR = (startR[0][0], startR[1][0])
		self.start = np.array([(self.startL[0]+self.startR[0])/2, (self.startL[1]+self.startR[1])/2])
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
				fnR = prog.NewContinuousVariables(self.dim+1, "fr" + str(fNum)) # Right Footstep
				fnL = prog.NewContinuousVariables(self.dim+1, "fl" + str(fNum)) # Left Footstep
				prog.AddLinearConstraint(fnR[2] <= 2*math.pi) # Angular constraints
				prog.AddLinearConstraint(fnR[2] >= 0)
				prog.AddLinearConstraint(fnL[2] <= 2*math.pi)
				prog.AddLinearConstraint(fnL[2] >= 0)
				f.append(fnR)
				f.append(fnL)
			n = prog.NewBinaryVariables(2*FootstepPlanner.MAXFOOTSTEPS, "n") # binary variables for nominal regions

			S = []
			s = prog.NewContinuousVariables(2*FootstepPlanner.MAXFOOTSTEPS, "s")
			for fNum in range(2*FootstepPlanner.MAXFOOTSTEPS):
				sf = prog.NewBinaryVariables(FootstepPlanner.NUM_LINEAR_PIECES, "sf" + str(fNum)) # binary variables for angular approximations
				prog.AddLinearConstraint(np.sum(sf)==1) # Each footstep is in only one region of [0, 2pi]

				# Set up angular bindings
				for region in range(FootstepPlanner.NUM_LINEAR_PIECES):
					prog.AddLinearConstraint(-f[fNum][2] + sf[region]* M <= -FootstepPlanner.PS[region] + M) # Lower Bound
					prog.AddLinearConstraint(f[fNum][2] + sf[region] * M <= FootstepPlanner.PS[region+1] + M) # Upper Bound
				# Bind sine approximation to the angle
				for region in range(FootstepPlanner.NUM_LINEAR_PIECES):
					prog.AddLinearConstraint(s[fNum] + sf[region]*M <= FootstepPlanner.GS[region] * f[fNum][2] + FootstepPlanner.HS[region] + M)
					prog.AddLinearConstraint(-s[fNum] + sf[region]*M <= -FootstepPlanner.GS[region] * f[fNum][2] - FootstepPlanner.HS[region] + M)

				S.append(sf)

			C = []
			c = prog.NewContinuousVariables(2*FootstepPlanner.MAXFOOTSTEPS, "c")
			for fNum in range(2*FootstepPlanner.MAXFOOTSTEPS):
				cf = prog.NewBinaryVariables(FootstepPlanner.NUM_LINEAR_PIECES, "cf" + str(fNum)) # binary variables for angular approximations
				prog.AddLinearConstraint(np.sum(cf)==1) # Each footstep is in only one region of [0, 2pi]

				# Set up angular bindings
				for region in range(FootstepPlanner.NUM_LINEAR_PIECES):
					prog.AddLinearConstraint(-f[fNum][2] + cf[region]* M <= -FootstepPlanner.PC[region] + M) # Lower Bound
					prog.AddLinearConstraint(f[fNum][2] + cf[region] * M <= FootstepPlanner.PC[region+1] + M) # Upper Bound
				# Bind cosine approximation to the angle
				for region in range(FootstepPlanner.NUM_LINEAR_PIECES):
					prog.AddLinearConstraint(c[fNum] + cf[region]*M <= FootstepPlanner.GC[region] * f[fNum][2] + FootstepPlanner.HC[region] + M)
					prog.AddLinearConstraint(-c[fNum] + cf[region]*M <= -FootstepPlanner.GC[region] * f[fNum][2] - FootstepPlanner.HC[region] + M)

				C.append(cf)

			# CONSTRAIN WITH REACHABLE REGIONS
			# Start position
			prog.AddLinearConstraint(f[0][0] == self.startR[0])
			prog.AddLinearConstraint(f[0][1] == self.startR[1])
			prog.AddLinearConstraint(f[0][2] == 0)
			# prog.AddLinearConstraint(c[0] == 1)
			# prog.AddLinearConstraint(s[0] == 0)
			prog.AddLinearConstraint(f[1][0] == self.startL[0])
			prog.AddLinearConstraint(f[1][1] == self.startL[1])
			prog.AddLinearConstraint(f[1][2] == 0)
			# prog.AddLinearConstraint(c[1] == 1)
			# prog.AddLinearConstraint(s[1] == 0)
			# All other footsteps
			for fNum in range(1, FootstepPlanner.MAXFOOTSTEPS):
					# Constrain angles (each left footstep can be within  (0, +pi/4) of previous right footstep; each right footstep can be within (0, -pi/4) of previous left footstep)
					prog.AddLinearConstraint(f[2*fNum][2] <= f[2*fNum-1][2])
					prog.AddLinearConstraint(f[2*fNum][2] >= f[2*fNum-1][2]-math.pi/4)
					prog.AddLinearConstraint(f[2*fNum+1][2] >= f[2*fNum][2])
					prog.AddLinearConstraint(f[2*fNum+1][2] <= f[2*fNum][2] + math.pi/4)

					# Constrain XY positions
					prog.AddLorentzConeConstraint(np.array([0*f[2*fNum][0]+self.r1, f[2*fNum][0]-(f[2*fNum-1][0]+(c[2*fNum-1]*self.c1[0]-s[2*fNum-1]*self.c1[1])), f[2*fNum][1]-(f[2*fNum-1][1]+(s[2*fNum-1]*self.c1[0]+c[2*fNum-1]*self.c1[1]))])) # Right Footstep (2*fNum) to previous left (2*fNum-1)
					prog.AddLorentzConeConstraint(np.array([0*f[2*fNum][0]+self.r2, f[2*fNum][0]-(f[2*fNum-1][0]+(c[2*fNum-1]*self.c2[0]-s[2*fNum-1]*self.c2[1])), f[2*fNum][1]-(f[2*fNum-1][1]+(s[2*fNum-1]*self.c2[0]+c[2*fNum-1]*self.c2[1]))])) #  by constraining it to be within the two circles
					prog.AddLorentzConeConstraint(np.array([0*f[2*fNum][0]+self.r1, f[2*fNum+1][0]-(f[2*fNum][0]-(c[2*fNum]*self.c1[0]-s[2*fNum]*self.c1[1])), f[2*fNum+1][1]-(f[2*fNum][1]-(s[2*fNum]*self.c1[0]+c[2*fNum]*self.c1[1]))])) # Left Footstep (2*fNum+1) to previous right (2*fNum)
					prog.AddLorentzConeConstraint(np.array([0*f[2*fNum][0]+self.r2, f[2*fNum+1][0]-(f[2*fNum][0]-(c[2*fNum]*self.c2[0]-s[2*fNum]*self.c2[1])), f[2*fNum+1][1]-(f[2*fNum][1]-(s[2*fNum]*self.c2[0]+c[2*fNum]*self.c2[1]))])) #  by constraining to be within the two circles
					if(self.hasNominal): # Nominal Regions
						pass
						# prog.AddLinearConstraint(self.reachableA[i][0]*(f[2*fNum][0]-f[2*fNum-1][0]) + self.reachableA[i][1]*(f[2*fNum][1]-(f[2*fNum-1][1]-self.yOffset)) + n[2*fNum]*M <= self.nominal*self.reachableb[i] + M) # Right Footstep (2*fNum) to previous left (2*fNum-1)
						# prog.AddLinearConstraint(self.reachableA[i][0]*(f[2*fNum+1][0]-f[2*fNum][0]) + self.reachableA[i][1]*(f[2*fNum+1][1]-(f[2*fNum][1]+self.yOffset)) + n[2*fNum+1]*M <= self.nominal*self.reachableb[i] + M) # Left Footstep (2*fNum+1) to previous right (2*fNum)

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
			prog.AddQuadraticCost(1000*((f[2*FootstepPlanner.MAXFOOTSTEPS-1][0]-self.goal[0])**2 + (f[2*FootstepPlanner.MAXFOOTSTEPS-1][1]-self.goal[1])**2))
			prog.AddQuadraticCost(1000*((f[2*FootstepPlanner.MAXFOOTSTEPS-2][0]-self.goal[0])**2 + (f[2*FootstepPlanner.MAXFOOTSTEPS-2][1]-self.goal[1])**2))

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
			self.sinApprox = prog.GetSolution(s)
			self.cosApprox = prog.GetSolution(c)
			self.footsteps = np.array(self.footsteps)
			self.upToDate = True

	def showSolution(self, save=False):
		print(str(self.numFootsteps) + " Footsteps:")
		for i in range(self.footsteps.shape[0]):
			print("[" + str(self.footsteps[i][0]) + ", " + str(self.footsteps[i][1]) + ", " + str(((180/math.pi) * self.footsteps[i][2])) + "]; (" + str(self.cosApprox[i]) + ", " + str(self.sinApprox[i]) + ")")

		if(not self.upToDate):
			print("Solution not up to date; please run solve() to update the solution")
		p = Plotter(self.dim, self.goal, self.footsteps, self.sinApprox, self.cosApprox, self.numFootsteps, self.c1, self.r1, self.c2, self.r2)
		if(self.hasNominal):
			p.setNominal(self.nominal)
		if(self.hasObstacleFree):
			p.setObstacleFree(self.oChulls)
		p.plot()
		if(save):
			p.save()
		p.show()
		