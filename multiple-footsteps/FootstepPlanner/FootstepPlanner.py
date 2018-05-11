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

# Julia
import julia

import math

# Make numpy round printed values
np.set_printoptions(suppress=True)

class FootstepPlanner:
	MAXFOOTSTEPS = 4
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
		self.solType = -1 # -1=outOfDate, 0=solveProgram, 1=solveProgram_CouenneCircle, 2=solveProgram_CouenneDiamond
		# Convex reachable regions based on previous footstep
		self.c1 = None # Circles
		self.c2 = None
		self.r1 = -1
		self.r2 = -1

		self.reachableA = None # Diamonds
		self.reachableb = None
		self.reachableChull = None
		self.offset = -1
		
		self.hasNominal = False # Nominal Regions
		self.nominal = -1
		# Obstacle Free Regions
		self.hasObstacleFree = False
		self.num_regions = -1
		self.oChulls = None
		self.oA = None
		self.ob = None

		# ADD JULIA OBJECT
		self.j = julia.Julia()
		self.j._call('using JuMP, AmplNLWriter')

	# Indicate that the solution is out of date
	def setOutOfDate(self):
		self.solType = -1

	# The reachable region is defined by two circles, given by their centers and radii
	def setReachableCircles(self, center1, radius1, center2, radius2):
		assert(isinstance(center1, list) and len(center1)==self.dim and isinstance(center2, list) and len(center2)==self.dim), "center1, center2 must be " + str(self.dim) + "D lists of coordinates; instead they are " + str(center1) + ", " + str(center2)
		assert(isinstance(radius1, float) and isinstance(radius2, float)), "radius1, radius2 need to be floats; instead they are " + str(type(radius1)) + ", " + str(type(radius2))
		self.c1 = center1
		self.r1 = radius1
		self.c2 = center2
		self.r2 = radius2
		self.setOutOfDate()

	# The reachable region is defined by a convex hull * centered at origin * at a given offset from the point (left of the right foot, right of the left foot)
	def setReachableDiamonds(self, pointSet, offset):
		assert(isinstance(pointSet, list)), "pointSet needs to be a list of points; instead it is " + str(type(pointSet))
		assert(isinstance(offset, float)), "offset needs to be a float; instead it is " + str(type(yOffset))
		self.reachableChull = ConvexHull(np.array(pointSet))
		self.reachableA = np.delete(self.reachableChull.equations, self.reachableChull.equations.shape[1]-1, 1)
		self.reachableb = -1*self.reachableChull.equations[:,self.reachableChull.equations.shape[1]-1]
		self.offset = offset
		self.setOutOfDate()

	def setNominal(self, nominalRatio):
		assert(isinstance(nominalRatio, float) and (0<nominalRatio<1)), "nominalRatio needs to be in (0,1): " + str(nominalRatio)
		self.nominal = nominalRatio
		self.hasNominal = True
		self.setOutOfDate()

	def setGoal(self, goal):
		assert(isinstance(goal, list) and len(goal)==self.dim), "goal needs to be a " + str(self.dim) + "-length list of coordinates; it is " + str(goal)
		self.goal = np.array(goal)
		self.setOutOfDate()

	def setStart(self, start):
		assert(isinstance(start, list) and len(start)==self.dim), "start needs to be a " + str(self.dim) + "-length list of coordinates; it is " + str(start)
		self.start = np.array(start)
		self.startL = (self.start[0], self.start[1]+0.12)
		self.startR = (self.start[0], self.start[1]-0.12)
		self.setOutOfDate()

	def setStartRL(self, startR, startL):
		# assert(isinstance(startL, list) and isinstance(startR, list) and len(startL)==self.dim and len(startR)==self.dim), "startL, startR need to be " + str(self.dim) + "-length tuples of coordinates; they are: " + type(startL) + ", " + type(startR)
		self.startL = (startL[0][0], startL[1][0])
		self.startR = (startR[0][0], startR[1][0])
		self.start = np.array([(self.startL[0]+self.startR[0])/2, (self.startL[1]+self.startR[1])/2])
		self.setOutOfDate()

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
		self.setOutOfDate()

	def solveProgram(self):
		if(self.solType != 0):
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
			# print("OBJECTIVE VALUES")
			# print(prog.Cost())

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
			self.solType = 0

	def solveProgram_CouenneCircle(self):
		if(self.solType != 1):
			self.j._call('m = Model(solver=AmplNLSolver("/home/rahuly/Data/Couenne-0.5.6/build/bin/couenne"))')
			M = 100
			
			# SET UP FOOTSTEP VARIABLES
			self.j._call('@variable(m, f[1:{},1:{}])'.format(2*FootstepPlanner.MAXFOOTSTEPS, self.dim+1)) # Create footstep variable array
			self.j._call('@constraint(m, [i=1:{}], f[i,{}] <= 2*pi)'.format(2*FootstepPlanner.MAXFOOTSTEPS, self.dim+1)) # Angular
			self.j._call('@constraint(m, [i=1:{}], f[i,{}] >= 0)'.format(2*FootstepPlanner.MAXFOOTSTEPS, self.dim+1))	 #  Constraints
			
			# CONSTRAIN WITH REACHABLE REGIONS
			# Starting Footsteps
			#  - Starting Right Footstep
			self.j._call('@constraint(m, f[1,1] == {})'.format(self.startR[0]))
			self.j._call('@constraint(m, f[1,2] == {})'.format(self.startR[1]))
			self.j._call('@constraint(m, f[1,3] == {})'.format(0))
			#  - Starting Left Footstep
			self.j._call('@constraint(m, f[2,1] == {})'.format(self.startL[0]))
			self.j._call('@constraint(m, f[2,2] == {})'.format(self.startL[1]))
			self.j._call('@constraint(m, f[2,3] == {})'.format(0))
			# All other footsteps
			#  - Angular Constraints
			self.j._call('@constraint(m, [i=3:{}; isodd(i)], f[i,3] <= f[i-1,3])'.format(2*FootstepPlanner.MAXFOOTSTEPS)) 	 # right footstep within 
			self.j._call('@constraint(m, [i=3:{}; isodd(i)], f[i,3] >= f[i-1,3]-pi/4)'.format(2*FootstepPlanner.MAXFOOTSTEPS)) #  (0, -pi/4) of prev left footstep
			self.j._call('@constraint(m, [i=3:{}; iseven(i)], f[i,3] >= f[i-1,3])'.format(2*FootstepPlanner.MAXFOOTSTEPS)) 	 # left footstep within
			self.j._call('@constraint(m, [i=3:{}; iseven(i)], f[i,3] <= f[i-1,3]+pi/4)'.format(2*FootstepPlanner.MAXFOOTSTEPS)) #  (0, +pi/4) of prev right footstep
			#  - Reachable Region Constraints
			self.j._call('@NLconstraint(m, [i=3:{}; isodd(i)], (f[i,1]-(f[i-1,1]+(cos(f[i-1,3])*{}-sin(f[i-1,3])*{})))^2 + (f[i,2]-(f[i-1,2]+(sin(f[i-1,3])*{}+cos(f[i-1,3])*{})))^2 <= {}^2)'.format(2*FootstepPlanner.MAXFOOTSTEPS, self.c1[0], self.c1[1], self.c1[0], self.c1[1], self.r1)) # Right Footstep (odd) to previous left: c1
			self.j._call('@NLconstraint(m, [i=3:{}; isodd(i)], (f[i,1]-(f[i-1,1]+(cos(f[i-1,3])*{}-sin(f[i-1,3])*{})))^2 + (f[i,2]-(f[i-1,2]+(sin(f[i-1,3])*{}+cos(f[i-1,3])*{})))^2 <= {}^2)'.format(2*FootstepPlanner.MAXFOOTSTEPS, self.c2[0], self.c2[1], self.c2[0], self.c2[1], self.r2)) # Right Footstep (odd) to previous left: c2
			self.j._call('@NLconstraint(m, [i=3:{}; iseven(i)], (f[i,1]-(f[i-1,1]-(cos(f[i-1,3])*{}-sin(f[i-1,3])*{})))^2 + (f[i,2]-(f[i-1,2]-(sin(f[i-1,3])*{}+cos(f[i-1,3])*{})))^2 <= {}^2)'.format(2*FootstepPlanner.MAXFOOTSTEPS, self.c1[0], self.c1[1], self.c1[0], self.c1[1], self.r1)) # Left Footstep (odd) to previous right: c1
			self.j._call('@NLconstraint(m, [i=3:{}; iseven(i)], (f[i,1]-(f[i-1,1]-(cos(f[i-1,3])*{}-sin(f[i-1,3])*{})))^2 + (f[i,2]-(f[i-1,2]-(sin(f[i-1,3])*{}+cos(f[i-1,3])*{})))^2 <= {}^2)'.format(2*FootstepPlanner.MAXFOOTSTEPS, self.c2[0], self.c2[1], self.c2[0], self.c2[1], self.r2)) # Left Footstep (odd) to previous right: c2

			self.j._call('@variable(m, n[1:{}], Bin)'.format(2*FootstepPlanner.MAXFOOTSTEPS)) # binary variables for nominal regions
			if(self.hasNominal): # Nominal Regions
				pass						
				# prog.AddLinearConstraint(self.reachableA[i][0]*(f[2*fNum][0]-f[2*fNum-1][0]) + self.reachableA[i][1]*(f[2*fNum][1]-(f[2*fNum-1][1]-self.yOffset)) + n[2*fNum]*M <= self.nominal*self.reachableb[i] + M) # Right Footstep (2*fNum) to previous left (2*fNum-1)
				# prog.AddLinearConstraint(self.reachableA[i][0]*(f[2*fNum+1][0]-f[2*fNum][0]) + self.reachableA[i][1]*(f[2*fNum+1][1]-(f[2*fNum][1]+self.yOffset)) + n[2*fNum+1]*M <= self.nominal*self.reachableb[i] + M) # Left Footstep (2*fNum+1) to previous right (2*fNum)

			# CONSTRAIN TO OBSTACLE FREE REGIONS
			if(self.hasObstacleFree):
				self.j._call('@variable(m, h[1:{},1:{}], Bin)'.format(2*FootstepPlanner.MAXFOOTSTEPS-2, self.num_regions)) # Create Binary indicator variables for each step after starting steps
				self.j._call('@constraint(m, [i=1:{}], sum(h[i,:])==1)'.format(2*FootstepPlanner.MAXFOOTSTEPS-2)) # Constraint each footstep to one region
				
				# Constrain the footsteps to the regions
				for i in range(self.num_regions):
					for j in range(self.oA[i].shape[0]):
						self.j._call('@constraint(m, [i=3:{}], {}*f[i,1]+{}*f[i,2]+{}*h[i,{}] <= {} + {})'.format(2*FootstepPlanner.MAXFOOTSTEPS-2, self.oA[i][j][0], self.oA[i][j][1], M, i+1, self.ob[i][j]),M)

			# OPTIMAL NUMBER OF FOOTSTEPS
			self.j._call('@variable(m, z[1:{}], Bin)'.format(FootstepPlanner.MAXFOOTSTEPS))
			self.j._call('@constraint(m, [i=1:{}], z[i] <= z[i+1])'.format(FootstepPlanner.MAXFOOTSTEPS-1))
			self.j._call('@constraint(m, z[{}]-z[1]==1)'.format(FootstepPlanner.MAXFOOTSTEPS))

			# if z[i], then the ith footstep is the same as the final footstep
			self.j._call('@constraint(m, [i=1:{}, j=1:2; isodd(i)], f[i,j]+{}*z[div(i+1,2)] <= f[{},j]+{})'.format(2*FootstepPlanner.MAXFOOTSTEPS, M, 2*FootstepPlanner.MAXFOOTSTEPS-1, M))
			self.j._call('@constraint(m, [i=1:{}, j=1:2; isodd(i)], -f[i,j]+{}*z[div(i+1,2)] <= -f[{},j]+{})'.format(2*FootstepPlanner.MAXFOOTSTEPS, M, 2*FootstepPlanner.MAXFOOTSTEPS-1, M))
			self.j._call('@constraint(m, [i=1:{}, j=1:2; iseven(i)], f[i,j]+{}*z[div(i,2)] <= f[{},j]+{})'.format(2*FootstepPlanner.MAXFOOTSTEPS, M, 2*FootstepPlanner.MAXFOOTSTEPS, M))
			self.j._call('@constraint(m, [i=1:{}, j=1:2; iseven(i)], -f[i,j]+{}*z[div(i,2)] <= -f[{},j]+{})'.format(2*FootstepPlanner.MAXFOOTSTEPS, M, 2*FootstepPlanner.MAXFOOTSTEPS, M))

			# ADD COSTS
			# Cost of consecutive footsteps (with nominal regions considered)
			self.j._call('cost = 0')
			for i in range(1, 2*FootstepPlanner.MAXFOOTSTEPS):
				self.j._call('cost = cost + (f[{},1]-f[{}+1,1])^2+(f[{},2]-f[{}+1,2])^2+2*(1-n[{}])'.format(i,i,i,i,i))

			# Cost of number of footsteps
			self.j._call('cost = cost - 5*sum(z)')

			# Cost of distance of final position to goal
			for i in range(2*FootstepPlanner.MAXFOOTSTEPS-1, 2*FootstepPlanner.MAXFOOTSTEPS+1):
				self.j._call('cost = cost + 1000*((f[{},1]-{})^2+(f[{},2]-{})^2)'.format(i, self.goal[0], i, self.goal[1]))
			
			# Add the overall cost
			self.j._call('@objective(m, Min, cost)')

			# SOLVE PROBLEM
			status = self.j.eval('solve(m)')
			print("****************************")
			print('Solve Status: {}'.format(status))

			# SAVE SOLUTION
			self.footsteps = self.j.eval('getvalue(f)')
			print("Footsteps Type: {}".format(type(self.footsteps)))
			z = self.j.eval('getvalue(z)')
			self.numFootsteps = 0
			for i in range(len(z)):
				self.numFootsteps +=1 
				if(z[i]==1):
					break
			print("Num Footsteps: {}".format(self.numFootsteps))
			self.footsteps = self.footsteps[:2*self.numFootsteps,:] # Cut redundant footsteps TODO: FIX THIS
			print("FOOTSTEPS")
			print(self.footsteps)
			self.sinApprox = np.sin(self.footsteps[:,2])
			print("SINE APPROX")
			print(self.sinApprox)
			self.cosApprox = np.cos(self.footsteps[:,2])
			print("COSINE APPROX")
			print(self.cosApprox)
			# self.footsteps = np.array(self.footsteps)

			self.solType = 1

	def solveProgram_CouenneDiamond(self):
		if(self.solType != 2):
			self.j._call('m = Model(solver=AmplNLSolver("/home/rahuly/Data/Couenne-0.5.6/build/bin/couenne"))')
			M = 100
			
			# SET UP FOOTSTEP VARIABLES
			self.j._call('@variable(m, f[1:{},1:{}])'.format(2*FootstepPlanner.MAXFOOTSTEPS, self.dim+1)) # Create footstep variable array
			self.j._call('@constraint(m, [i=1:{}], f[i,{}] <= 2*pi)'.format(2*FootstepPlanner.MAXFOOTSTEPS, self.dim+1)) # Angular
			self.j._call('@constraint(m, [i=1:{}], f[i,{}] >= 0)'.format(2*FootstepPlanner.MAXFOOTSTEPS, self.dim+1))	 #  Constraints
			
			# CONSTRAIN WITH REACHABLE REGIONS
			# Starting Footsteps
			#  - Starting Right Footstep
			self.j._call('@constraint(m, f[1,1] == {})'.format(self.startR[0]))
			self.j._call('@constraint(m, f[1,2] == {})'.format(self.startR[1]))
			self.j._call('@constraint(m, f[1,3] == {})'.format(0))
			#  - Starting Left Footstep
			self.j._call('@constraint(m, f[2,1] == {})'.format(self.startL[0]))
			self.j._call('@constraint(m, f[2,2] == {})'.format(self.startL[1]))
			self.j._call('@constraint(m, f[2,3] == {})'.format(0))
			# All other footsteps
			#  - Angular Constraints
			self.j._call('@constraint(m, [i=3:{}; isodd(i)], f[i,3] <= f[i-1,3])'.format(2*FootstepPlanner.MAXFOOTSTEPS)) 	 # right footstep within 
			self.j._call('@constraint(m, [i=3:{}; isodd(i)], f[i,3] >= f[i-1,3]-pi/4)'.format(2*FootstepPlanner.MAXFOOTSTEPS)) #  (0, -pi/4) of prev left footstep
			self.j._call('@constraint(m, [i=3:{}; iseven(i)], f[i,3] >= f[i-1,3])'.format(2*FootstepPlanner.MAXFOOTSTEPS)) 	 # left footstep within
			self.j._call('@constraint(m, [i=3:{}; iseven(i)], f[i,3] <= f[i-1,3]+pi/4)'.format(2*FootstepPlanner.MAXFOOTSTEPS)) #  (0, +pi/4) of prev right footstep
			
			#  - Reachable Region Constraints
			for i in range(self.reachableA.shape[0]): # constraint each footstep to each piece of the reachable region
				# ([{} {}] * ([cos(f[i,3]) sin(f[i,3]); -sin(f[i,3]) cos(f[i,3])] * ([f[i,1];f[i,2]]-[f[i-1,1],f[i-1,2]]) - [0; (-1)^i*{}]))[1] <= {}
				self.j._call('@NLconstraint(m, [i=3:{}], {}*(cos(f[i-1,3])*(f[i,1]-f[i-1,1])+sin(f[i-1,3])*(f[i,2]-f[i-1,2])) + {}*(-sin(f[i-1,3])*(f[i,1]-f[i-1,1])+cos(f[i-1,3])*(f[i,2]-f[i-1,2])-(-1)^i*{}) <= {})'.format(2*FootstepPlanner.MAXFOOTSTEPS, self.reachableA[i][0], self.reachableA[i][1], self.offset, self.reachableb[i]))

			self.j._call('@variable(m, n[1:{}], Bin)'.format(2*FootstepPlanner.MAXFOOTSTEPS)) # binary variables for nominal regions
			if(self.hasNominal): # Nominal Regions
				pass						
				# prog.AddLinearConstraint(self.reachableA[i][0]*(f[2*fNum][0]-f[2*fNum-1][0]) + self.reachableA[i][1]*(f[2*fNum][1]-(f[2*fNum-1][1]-self.yOffset)) + n[2*fNum]*M <= self.nominal*self.reachableb[i] + M) # Right Footstep (2*fNum) to previous left (2*fNum-1)
				# prog.AddLinearConstraint(self.reachableA[i][0]*(f[2*fNum+1][0]-f[2*fNum][0]) + self.reachableA[i][1]*(f[2*fNum+1][1]-(f[2*fNum][1]+self.yOffset)) + n[2*fNum+1]*M <= self.nominal*self.reachableb[i] + M) # Left Footstep (2*fNum+1) to previous right (2*fNum)

			# CONSTRAIN TO OBSTACLE FREE REGIONS
			if(self.hasObstacleFree):
				self.j._call('@variable(m, h[1:{},1:{}], Bin)'.format(2*FootstepPlanner.MAXFOOTSTEPS-2, self.num_regions)) # Create Binary indicator variables for each step after starting steps
				self.j._call('@constraint(m, [i=1:{}], sum(h[i,:])==1)'.format(2*FootstepPlanner.MAXFOOTSTEPS-2)) # Constraint each footstep to one region
				
				# Constrain the footsteps to the regions
				for i in range(self.num_regions):
					for j in range(self.oA[i].shape[0]):
						self.j._call('@constraint(m, [i=3:{}], {}*f[i,1]+{}*f[i,2]+{}*h[i,{}] <= {} + {})'.format(2*FootstepPlanner.MAXFOOTSTEPS-2, self.oA[i][j][0], self.oA[i][j][1], M, i+1, self.ob[i][j]),M)

			# OPTIMAL NUMBER OF FOOTSTEPS
			self.j._call('@variable(m, z[1:{}], Bin)'.format(FootstepPlanner.MAXFOOTSTEPS))
			self.j._call('@constraint(m, [i=1:{}], z[i] <= z[i+1])'.format(FootstepPlanner.MAXFOOTSTEPS-1))
			self.j._call('@constraint(m, z[{}]-z[1]==1)'.format(FootstepPlanner.MAXFOOTSTEPS))

			# if z[i], then the ith footstep is the same as the final footstep
			self.j._call('@constraint(m, [i=1:{}, j=1:2; isodd(i)], f[i,j]+{}*z[div(i+1,2)] <= f[{},j]+{})'.format(2*FootstepPlanner.MAXFOOTSTEPS, M, 2*FootstepPlanner.MAXFOOTSTEPS-1, M))
			self.j._call('@constraint(m, [i=1:{}, j=1:2; isodd(i)], -f[i,j]+{}*z[div(i+1,2)] <= -f[{},j]+{})'.format(2*FootstepPlanner.MAXFOOTSTEPS, M, 2*FootstepPlanner.MAXFOOTSTEPS-1, M))
			self.j._call('@constraint(m, [i=1:{}, j=1:2; iseven(i)], f[i,j]+{}*z[div(i,2)] <= f[{},j]+{})'.format(2*FootstepPlanner.MAXFOOTSTEPS, M, 2*FootstepPlanner.MAXFOOTSTEPS, M))
			self.j._call('@constraint(m, [i=1:{}, j=1:2; iseven(i)], -f[i,j]+{}*z[div(i,2)] <= -f[{},j]+{})'.format(2*FootstepPlanner.MAXFOOTSTEPS, M, 2*FootstepPlanner.MAXFOOTSTEPS, M))

			# ADD COSTS
			# Cost of consecutive footsteps (with nominal regions considered)
			self.j._call('cost = 0')
			for i in range(1, 2*FootstepPlanner.MAXFOOTSTEPS):
				self.j._call('cost = cost + (f[{},1]-f[{}+1,1])^2+(f[{},2]-f[{}+1,2])^2+2*(1-n[{}])'.format(i,i,i,i,i))

			# Cost of number of footsteps
			self.j._call('cost = cost - 5*sum(z)')

			# Cost of distance of final position to goal
			for i in range(2*FootstepPlanner.MAXFOOTSTEPS-1, 2*FootstepPlanner.MAXFOOTSTEPS+1):
				self.j._call('cost = cost + 1000*((f[{},1]-{})^2+(f[{},2]-{})^2)'.format(i, self.goal[0], i, self.goal[1]))
			
			# Add the overall cost
			self.j._call('@objective(m, Min, cost)')

			# SOLVE PROBLEM
			status = self.j.eval('solve(m)')
			print("****************************")
			print('Solve Status: {}'.format(status))

			# SAVE SOLUTION
			self.footsteps = self.j.eval('getvalue(f)')
			print("Footsteps Type: {}".format(type(self.footsteps)))
			z = self.j.eval('getvalue(z)')
			self.numFootsteps = 0
			for i in range(len(z)):
				self.numFootsteps +=1 
				if(z[i]==1):
					break
			print("Num Footsteps: {}".format(self.numFootsteps))
			self.footsteps = self.footsteps[:2*self.numFootsteps,:] # Cut redundant footsteps TODO: FIX THIS
			print("FOOTSTEPS")
			print(self.footsteps)
			self.sinApprox = np.sin(self.footsteps[:,2])
			print("SINE APPROX")
			print(self.sinApprox)
			self.cosApprox = np.cos(self.footsteps[:,2])
			print("COSINE APPROX")
			print(self.cosApprox)
			# self.footsteps = np.array(self.footsteps)

			self.solType = 2

	def showSolution(self, save=False):
		# Print Footsteps and Angles (Sine + Cosine)
		print(str(self.numFootsteps) + " Footsteps:")
		for i in range(self.footsteps.shape[0]):
			print("[" + str(self.footsteps[i][0]) + ", " + str(self.footsteps[i][1]) + ", " + str(((180/math.pi) * self.footsteps[i][2])) + "]; (" + str(self.cosApprox[i]) + ", " + str(self.sinApprox[i]) + ")")

		# Check whether solution is up to date
		if(self.solType<0):
			raise ValueError("Solution not up to date; please run solve() to update the solution")

		# Create Plotter object
		p = Plotter(self.dim, self.goal, self.footsteps, self.sinApprox, self.cosApprox, self.numFootsteps)

		# Pass reachable regions based on solution type
		if(self.solType == 2):
			p.setType('DIAMOND')
			p.setReachableDiamonds(self.reachableChull, self.offset)
		else:
			p.setType('CIRCLE')
			p.setReachableCircles(self.c1, self.r1, self.c2, self.r2)

		# Set nominal regions
		if(self.hasNominal):
			p.setNominal(self.nominal)

		# Set Obstacle Free regions
		if(self.hasObstacleFree):
			p.setObstacleFree(self.oChulls)

		# Plot regions
		p.plot()

		# Save/Show
		if(save):
			p.save()
		p.show()
		