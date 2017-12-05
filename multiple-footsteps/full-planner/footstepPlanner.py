from __future__ import absolute_import

# Used by Plotter
import sys
import os 
import string
import random

# numpy/scipy imports
import numpy as np # numpy for matrix/vector manipulation
from scipy.spatial import ConvexHull # ConvexHull function

# matplotlib for plotting footstep plans
import matplotlib.pyplot as plt # pyplot
# import matplotlib.patches as patches # patches allow us to make smaller patches
# from mpl_toolkits.mplot3d import Axes3D # 3D plotting

# Pydrake imports
import pydrake # basic pydrake commands
import pydrake.symbolic as sym # symbolic variables
from pydrake.solvers import mathematicalprogram as mp # mathematical program interface
from pydrake.solvers.gurobi import GurobiSolver # Gurobi as solver

class Plotter:
	totalFigures = 0
	def __init__(dim, goal, footsteps, numFootsteps, open_chulls=None, open_A=None, open_b=None):
		# ---- VALIDATE VARIABLES ----
		# validate footsteps
		assert(dim==2 or dim==3), "dim: must be either 2D or 3D"
		assert(isinstance(goal, np.array) and goal.shape[0]==dim), "goal: must be an dim-dimension point in a numpy.array"
		assert(isinstance(footsteps, np.array) and footsteps.shape[1]==dim), "footsteps: must be a numpy.array of dim-dimension points"
		assert(isinstance(numFootsteps, int) and numFootsteps<=footsteps.shape[0]), "numFootsteps: must be <= the total number of footsteps in footsteps"
		# validate obstacle-free convex regions
		assert(isinstance(open_chulls, np.array) or open_chulls==None), "open_chulls: must be None or a numpy.array"
		assert(isinstance(open_A, np.array) or open_A==None), "open_A: must be None or a numpy.array"
		assert(isinstance(open_b, np.array) or open_b==None), "open_b: must be None or a numpy.array"
		if(open_chulls==None or open_A==None or open_b==None):
			assert(open_chulls==None and open_A==None and open_b==None), "open_chulls, open_A, open_b: must either all be None or all be defined"

		# ---- SAVE VARIABLES ----
		self.dim = dim
		self.goal = goal
		self.footsteps = footsteps
		self.numFootsteps = numFootsteps
		self.has_open = (open_chulls!=None) # boolean whether the plotter should show obstacle-free regions
		self.open_chulls = open_chulls
		self.open_A	= open_A
		self.open_b = open_b
		# TODO: Add convex reachable regions
		totalFigures+=1
		self.figNum = totalFigures

	def plot():
		# make figure
		fig = plt.figure(self.figNum, (20, 10))
		ax = None
		if(dim==3):
			ax = fig.add_subplot(111, projection='3d')

		# add titles
		plt.title("Footstep Plan: " + str(numFootsteps) + " Footsteps in " + str(dim) + "D")
		plt.xlabel("X Displacement")
		plt.ylabel("Y Displacement")

		# plot goal
		plt.plot(*goal, 'bo')

		# plot footsteps
		pass

		# plot obstacle free regions
		if(self.has_open):
			pass

	def show():
		plt.figure(self.figNum, (20, 10))
		plt.show()

	def save():
		name = self.name_gen()
		fig = plt.figure(self.figNum, (20, 10))
		fig.savefig(name, bbox_inches='tight')

	def name_gen():
		idsize=6
		chars=string.ascii_uppercase+string.digits:
		dir_path = os.path.dirname(os.path.realpath(__file__)) # file where script is stored
		results_dir = os.path.join(dir_path, 'images/') # images subfolder
		file_name = str(dim) + 'D-' + ''.join(random.choice(chars) for _ in range(idsize)) + '.png' # create file name with random ID
		if not os.path.isdir(results_dir): # make the images/ folder if it doesn't exist
			os.makedirs(results_dir)
		return results_dir + file_name


if __name__ == '__main__':
	# Make numpy round printed values
	np.set_printoptions(suppress=True)