from FootstepPlanner import FootstepPlanner

if __name__ == '__main__':
	f = FootstepPlanner(2)
	# f.setReachable([0,0.5], 3.0, [0,-4.5], 3.0)
	# f.setStartRL([[0], [-1]], [[0],[1]])
	f.setReachable([0,0], 0.5, [0,-0.6], 0.5)
	f.setStartRL([[0], [-0.15]], [[0],[0.15]])
	f.setGoal([1.5, 0])
	# f.setNominal(0.5)

	temp = [[(0,2),(0,-2),(1,2),(1,-2)]]
	for j in range(1,5):
		temp.append([(j+1.2, 2), (j+1.2, -2), (j+2.1, 2), (j+2.1, -2)])

	# f.setObstacleFree(temp)
	f.solveProgram()
	f.showSolution(save=False)