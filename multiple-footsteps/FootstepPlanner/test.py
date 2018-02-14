from FootstepPlanner import FootstepPlanner

if __name__ == '__main__':
	f = FootstepPlanner(2)
	f.setReachable([0,0.75], 1.5, [0,-0.75], 1.5)
	f.setStart([0, 0])
	f.setGoal([10, 10])
	f.setNominal(0.5)

	temp = [[(0,2),(0,-2),(1,2),(1,-2)]]
	for j in range(1,5):
		temp.append([(j+1.2, 2), (j+1.2, -2), (j+2.1, 2), (j+2.1, -2)])

	f.setObstacleFree(temp)
	f.solveProgram()
	f.showSolution(save=True)