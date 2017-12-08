from __future__ import absolute_import

# ID-generation imports
import sys
import os 
import string
import random

# numpy/scipy imports
import numpy as np # numpy for matrix/vector manipulation

# matplotlib for plotting footstep plans
import matplotlib.pyplot as plt # pyplot
from mpl_toolkits.mplot3d import Axes3D # 3D plotting

np.set_printoptions(suppress=True)

class Plotter:
	totalFigures = 0
	def __init__(self, dim, goal, footsteps, numFootsteps, open_chulls=None, open_A=None, open_b=None):
		# ---- VALIDATE VARIABLES ----
		# validate footsteps
		assert(dim==2 or dim==3), "dim: must be either 2D or 3D; it is " + str(dim)
		assert(isinstance(goal, np.ndarray) and goal.shape[0]==dim), "goal: must be an dim-dimension point in a numpy.array"
		assert(isinstance(footsteps, np.ndarray) and footsteps.shape[1]==dim), "footsteps: must be a numpy.array of " + str(dim) + "-dimension points; instead footsteps.shape[1] = " + str(footsteps.shape[1])
		assert(isinstance(numFootsteps, int) and numFootsteps<=footsteps.shape[0]), "numFootsteps: must be <= the total number of footsteps in footsteps"
		# validate obstacle-free convex regions
		assert(isinstance(open_chulls, np.ndarray) or open_chulls==None), "open_chulls: must be None or a numpy.array"
		assert(isinstance(open_A, np.ndarray) or open_A==None), "open_A: must be None or a numpy.array"
		assert(isinstance(open_b, np.ndarray) or open_b==None), "open_b: must be None or a numpy.array"
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
		Plotter.totalFigures+=1
		self.figNum = Plotter.totalFigures

	def plot(self):
		# make figure
		fig = plt.figure(self.figNum, (20, 10))
		ax = None
		if(self.dim==3):
			ax = fig.add_subplot(111, projection='3d')

		# add titles
		plt.title("Footstep Plan: " + str(self.numFootsteps) + " Footsteps in " + str(self.dim) + "D")
		plt.xlabel("X Displacement")
		plt.ylabel("Y Displacement")

		# plot goal
		plt.plot(*self.goal, color='blue', marker='o')

		# plot footsteps
		for i in range(self.numFootsteps):
			plt.plot(*self.footsteps[2*i], color='green', marker='o') # Left footstep
			plt.plot(*self.footsteps[2*i+1], color='red', marker='o') # Right footstep

		# plot obstacle free regions
		if(self.has_open):
			pass

	def show(self):
		plt.figure(self.figNum, (20, 10))
		plt.show()

	def save(self):
		name = self.name_gen()
		fig = plt.figure(self.figNum, (20, 10))
		fig.savefig(name, bbox_inches='tight')

	def name_gen(self):
		idsize=6
		chars=string.ascii_uppercase+string.digits
		dir_path = os.path.dirname(os.path.realpath(__file__)) # file where script is stored
		results_dir = os.path.join(dir_path, 'images/') # images subfolder
		file_name = str(self.dim) + 'D-' + ''.join(random.choice(chars) for _ in range(idsize)) + '.png' # create file name with random ID
		if not os.path.isdir(results_dir): # make the images/ folder if it doesn't exist
			os.makedirs(results_dir)
		return results_dir + file_name


if __name__ == '__main__':
	print("Why is Plotter running?")
	