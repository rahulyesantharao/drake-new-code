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
import shapely.geometry as sg
import descartes

# ConvexHull
from scipy.spatial import ConvexHull

import math

np.set_printoptions(suppress=True)

class Plotter:
	totalFigures = 0
	ARROW_LENGTH = 0.25
	def __init__(self, dim, goal, footsteps, sinApprox, cosApprox, numFootsteps, c1, r1, c2, r2):
		# ---- VALIDATE VARIABLES ----
		assert(dim==2 or dim==3), "dim: must be either 2D or 3D; it is " + str(dim)
		assert(isinstance(goal, np.ndarray) and goal.shape[0]==dim), "goal: must be an dim-dimension point in a numpy.array"
		assert(isinstance(footsteps, np.ndarray) and footsteps.shape[1]==dim+1), "footsteps: must be a numpy.array of " + str(dim)+1 + "-dimension points; instead footsteps.shape[1] = " + str(footsteps.shape[1])
		assert(isinstance(sinApprox, np.ndarray) and sinApprox.shape[0] >= 2*numFootsteps), "sinApprox must be a numpy.array; instead, it is " + str(type(sinApprox)) + " with length " + str(sinApprox.shape[0])
		assert(isinstance(cosApprox, np.ndarray) and cosApprox.shape[0] >= 2*numFootsteps), "cosApprox must be a numpy.array; instead, it is " + str(type(cosApprox)) + " with length " + str(cosApprox.shape[0])
		assert(isinstance(numFootsteps, int) and numFootsteps<=footsteps.shape[0]), "numFootsteps: must be <= the total number of footsteps in footsteps"
		assert(isinstance(c1, list) and len(c1)==dim), "c1 must be a list of length " + str(dim) + "; it is " + str(c1)
		assert(isinstance(r1, float)), "r1 must be a float; it is " + str(r1)
		assert(isinstance(c2, list) and len(c2)==dim), "c2 must be a list of length " + str(dim) + "; it is " + str(c2)
		assert(isinstance(r2, float)), "r2 must be a float; it is " + str(r2)
		self.upToDate = False

		# Footstep variables
		self.dim = dim
		self.goal = goal
		self.footsteps = footsteps
		self.sinApprox = sinApprox
		self.cosApprox = cosApprox
		self.numFootsteps = numFootsteps
		# Convex Reachable Regions
		self.c1 = c1
		self.r1 = r1
		self.c2 = c2
		self.r2 = r2
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

			plt.gca().set_aspect('equal', adjustable='box')
			
			# plot goal
			plt.plot(*self.goal, color='blue', marker='o')

			# plot footsteps
			for i in range(self.numFootsteps):				
				# RIGHT FOOTSTEP
				print("Right: " + str(self.footsteps[2*i]))
				print("C1: (" + str(round(self.footsteps[2*i][0] - (self.cosApprox[2*i]*self.c1[0]-self.sinApprox[2*i]*self.c1[1]),2)) + ", "  + str(round(self.footsteps[2*i][1] - (self.sinApprox[2*i]*self.c1[0]+self.cosApprox[2*i]*self.c1[1]),2)) + ")")
				print("C2: (" + str(round(self.footsteps[2*i][0] - (self.cosApprox[2*i]*self.c2[0]-self.sinApprox[2*i]*self.c2[1]),2)) + ", "  + str(round(self.footsteps[2*i][1] - (self.sinApprox[2*i]*self.c2[0]+self.cosApprox[2*i]*self.c2[1]),2)) + ")")

				plt.plot([self.footsteps[2*i][0], self.footsteps[2*i][0]+Plotter.ARROW_LENGTH*math.cos(self.footsteps[2*i][2])],[self.footsteps[2*i][1], self.footsteps[2*i][1]+Plotter.ARROW_LENGTH*math.sin(self.footsteps[2*i][2])], color='green') # footstep
				plt.annotate("R"+str(i) + ": " + str(self.footsteps[2*i][2]*180/math.pi), xy=[self.footsteps[2*i][0],self.footsteps[2*i][1]]) # annotation
				# reachable region
				rcircle1 = sg.Point(self.footsteps[2*i][0] - (self.cosApprox[2*i]*self.c1[0]-self.sinApprox[2*i]*self.c1[1]), self.footsteps[2*i][1] - (self.sinApprox[2*i]*self.c1[0]+self.cosApprox[2*i]*self.c1[1])).buffer(self.r1)
				rcircle2 = sg.Point(self.footsteps[2*i][0] - (self.cosApprox[2*i]*self.c2[0]-self.sinApprox[2*i]*self.c2[1]), self.footsteps[2*i][1] - (self.sinApprox[2*i]*self.c2[0]+self.cosApprox[2*i]*self.c2[1])).buffer(self.r2)
				rreachable = rcircle1.intersection(rcircle2)
				ax = plt.gca()
				ax.add_patch(descartes.PolygonPatch(rreachable, fc='g', ec='k', alpha=0.2))
				# for simplex in self.reachable_chull.simplices: # Region reachable from footstep
					# plt.plot(self.reachable_chull.points[simplex, 0] + self.footsteps[2*i][0], self.reachable_chull.points[simplex, 1] + (self.footsteps[2*i][1]+self.yOffset), color='green', alpha=0.15)
				if(self.hasNominal):	
					pass
					# for simplex in self.reachable_chull.simplices: # nominal region from footstep
						# plt.plot(self.nominalRatio*self.reachable_chull.points[simplex, 0] + self.footsteps[2*i][0], self.nominalRatio*self.reachable_chull.points[simplex, 1] + (self.footsteps[2*i][1]+self.yOffset), color='green', alpha=0.3)

				# LEFT FOOTSTEP
				print("Left: " + str(self.footsteps[2*i+1]))
				print("C1: (" + str(round(self.footsteps[2*i+1][0] + (self.cosApprox[2*i+1]*self.c1[0]-self.sinApprox[2*i+1]*self.c1[1]),2)) + ", "  + str(round(self.footsteps[2*i+1][1] + (self.sinApprox[2*i+1]*self.c1[0]+self.cosApprox[2*i+1]*self.c1[1]),2)) + ")")
				print("C2: (" + str(round(self.footsteps[2*i+1][0] + (self.cosApprox[2*i+1]*self.c2[0]-self.sinApprox[2*i+1]*self.c2[1]),2)) + ", "  + str(round(self.footsteps[2*i+1][1] + (self.sinApprox[2*i+1]*self.c2[0]+self.cosApprox[2*i+1]*self.c2[1]),2)) + ")")

				plt.plot([self.footsteps[2*i+1][0], self.footsteps[2*i+1][0]+Plotter.ARROW_LENGTH*math.cos(self.footsteps[2*i+1][2])],[self.footsteps[2*i+1][1], self.footsteps[2*i+1][1]+Plotter.ARROW_LENGTH*math.sin(self.footsteps[2*i+1][2])], color='red') # footstep
				plt.annotate("L"+str(i) + ": " + str(self.footsteps[2*i+1][2]*180/math.pi), xy=[self.footsteps[2*i+1][0], self.footsteps[2*i+1][1]]) # annotation
				# reachable region
				lcircle1 = sg.Point(self.footsteps[2*i+1][0] + (self.cosApprox[2*i+1]*self.c1[0]-self.sinApprox[2*i+1]*self.c1[1]), self.footsteps[2*i+1][1] + (self.sinApprox[2*i+1]*self.c1[0]+self.cosApprox[2*i+1]*self.c1[1])).buffer(self.r1)
				lcircle2 = sg.Point(self.footsteps[2*i+1][0] + (self.cosApprox[2*i+1]*self.c2[0]-self.sinApprox[2*i+1]*self.c2[1]), self.footsteps[2*i+1][1] + (self.sinApprox[2*i+1]*self.c2[0]+self.cosApprox[2*i+1]*self.c2[1])).buffer(self.r2)
				lreachable = lcircle1.intersection(lcircle2)
				ax = plt.gca()
				ax.add_patch(descartes.PolygonPatch(lreachable, fc='r', ec='k', alpha=0.2))
				# for simplex in self.reachable_chull.simplices: # Region reachable from footstep
					# plt.plot(self.reachable_chull.points[simplex, 0] + self.footsteps[2*i+1][0], self.reachable_chull.points[simplex, 1] + (self.footsteps[2*i+1][1]-self.yOffset), color='red', alpha=0.75)
				if(self.hasNominal):
					pass
					# for simplex in self.reachable_chull.simplices: # nominal region from footstep
						# plt.plot(self.nominalRatio*self.reachable_chull.points[simplex, 0] + self.footsteps[2*i+1][0], self.nominalRatio*self.reachable_chull.points[simplex, 1] + (self.footsteps[2*i+1][1]-self.yOffset), color='red', alpha=0.75)

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
		while(os.path.isfile(name)):
			name = self.name_gen() # Make sure file does not already exist
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
	