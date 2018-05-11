from FootstepPlanner import FootstepPlanner
import sys

if __name__ == '__main__':
	f = FootstepPlanner(2)
	# f.setReachable([0,0.5], 3.0, [0,-4.5], 3.0)
	# f.setStartRL([[0], [-1]], [[0],[1]])
	f.setReachableCircles([0,0], 0.5, [0,-0.6], 0.5)
	f.setReachableDiamonds([(0,-0.15), (0.30,0), (0,0.15), (-0.30,0)], 0.30)
	f.setStartRL([[0], [-0.15]], [[0],[0.15]])
	f.setGoal([1.5, 1.2])
	# f.setNominal(0.5)

	temp = [[(0,2),(0,-2),(1,2),(1,-2)]]
	for j in range(1,5):
		temp.append([(j+1.2, 2), (j+1.2, -2), (j+2.1, 2), (j+2.1, -2)])

	# f.setObstacleFree(temp)
	if(len(sys.argv)>1):
		if(sys.argv[1]=='couenne-circle'):
			f.solveProgram_CouenneCircle()
		elif(sys.argv[1]=='couenne-diamond'):
			f.solveProgram_CouenneDiamond()
		else:
			raise ValueError("Usage: python test.py couenne-<diamond/circle>")
	else:
		f.solveProgram()
	f.showSolution(save=True)