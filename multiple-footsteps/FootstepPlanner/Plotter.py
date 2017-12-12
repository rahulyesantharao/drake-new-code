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

# ConvexHull
from scipy.spatial import ConvexHull

np.set_printoptions(suppress=True)

class Plotter:
	totalFigures = 0
	def __init__(self, dim, goal, footsteps, numFootsteps, reachable_chull, yOffset):
		# ---- VALIDATE VARIABLES ----
		assert(dim==2 or dim==3), "dim: must be either 2D or 3D; it is " + str(dim)
		assert(isinstance(goal, np.ndarray) and goal.shape[0]==dim), "goal: must be an dim-dimension point in a numpy.array"
		assert(isinstance(footsteps, np.ndarray) and footsteps.shape[1]==dim), "footsteps: must be a numpy.array of " + str(dim) + "-dimension points; instead footsteps.shape[1] = " + str(footsteps.shape[1])
		assert(isinstance(numFootsteps, int) and numFootsteps<=footsteps.shape[0]), "numFootsteps: must be <= the total number of footsteps in footsteps"
		assert(isinstance(reachable_chull, ConvexHull)), "reachable_chull must be a ConvexHull; instead: " + str(type(reachable_chull))
		assert(isinstance(yOffset, float)), "yOffset must be a float; it is: " + str(type(yOffset))
		self.upToDate = False

		# Footstep variables
		self.dim = dim
		self.goal = goal
		self.footsteps = footsteps
		self.numFootsteps = numFootsteps
		# Convex Reachable Regions
		self.reachable_chull = reachable_chull
		self.yOffset = yOffset
		self.hasNominal = False
		self.nominalRatio = -1
		# Convex Obstacle-Free Regions
		self.has_open = False # boolean whether the plotter should show obstacle-free regions
		self.open_chulls = None
		self.num_open_regions = -1
		# Set Figure Number
		Plotter.totalFigures+=1
		self.figNum = Plotter.totalFigures

	def setObstacleFree(self, regions):
		assert(isinstance(regions, list)), "regions must be a list of ConvexHulls; it is " + str(type(regions))
		self.open_chulls = regions
		self.num_open_regions = len(regions)
		self.has_open = True
		self.upToDate = False

	def setNominal(self, nominalRatio):
		assert(isinstance(nominalRatio, float) and 0<nominalRatio<1), "nominalRatio must be in (0,1): " + str(nominalRatio)
		self.nominalRatio = nominalRatio
		self.hasNominal = True
		self.upToDate = False

	def plot(self):
		if(not self.upToDate):
			# make figure
			fig = plt.figure(self.figNum, (20, 10))
			fig.clear()
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
				print("Right: " + str(self.footsteps[2*i]))
				print("Left: " + str(self.footsteps[2*i+1]))
				
				# RIGHT FOOTSTEP
				plt.plot(*self.footsteps[2*i], color='green', marker='o') # footstep
				plt.annotate("R"+str(i), xy=self.footsteps[2*i]) # annotation
				for simplex in self.reachable_chull.simplices: # Region reachable from footstep
					plt.plot(self.reachable_chull.points[simplex, 0] + self.footsteps[2*i][0], self.reachable_chull.points[simplex, 1] + (self.footsteps[2*i][1]+self.yOffset), color='green', alpha=0.15)
				if(self.hasNominal):	
					for simplex in self.reachable_chull.simplices: # nominal region from footstep
						plt.plot(self.nominalRatio*self.reachable_chull.points[simplex, 0] + self.footsteps[2*i][0], self.nominalRatio*self.reachable_chull.points[simplex, 1] + (self.footsteps[2*i][1]+self.yOffset), color='green', alpha=0.3)

				# LEFT FOOTSTEP
				plt.plot(*self.footsteps[2*i+1], color='red', marker='o') # footstep
				plt.annotate("L"+str(i), xy=self.footsteps[2*i+1]) # annotation
				for simplex in self.reachable_chull.simplices: # Region reachable from footstep
					plt.plot(self.reachable_chull.points[simplex, 0] + self.footsteps[2*i+1][0], self.reachable_chull.points[simplex, 1] + (self.footsteps[2*i+1][1]-self.yOffset), color='red', alpha=0.75)
				if(self.hasNominal): # nominal region from footstep
					for simplex in self.reachable_chull.simplices: # nominal region from Right footstep
						plt.plot(self.nominalRatio*self.reachable_chull.points[simplex, 0] + self.footsteps[2*i+1][0], self.nominalRatio*self.reachable_chull.points[simplex, 1] + (self.footsteps[2*i+1][1]-self.yOffset), color='red', alpha=0.75)

			# plot obstacle free regions
			if(self.has_open):
				# Plot regions
				for j in range(self.num_open_regions):
					# print("Region " + str(j))
					for simplex in self.open_chulls[j].simplices:
						# print(simplex)
						# print(chulls[j].points[simplex])
						plt.plot(self.open_chulls[j].points[simplex, 0], self.open_chulls[j].points[simplex, 1], 'b')


			self.upToDate = True

	def show(self):
		if(not self.upToDate):
			print("Not up to date; run plot() to update")
		plt.figure(self.figNum, (20, 10))
		plt.show()

	def save(self):
		if(not self.upToDate):
			print("Not up to date; run plot() to update")
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
	