from __future__ import absolute_import

import pydrake
import numpy as np
from pydrake.solvers import mathematicalprogram as mp
import pydrake.symbolic as sym


if __name__ == '__main__':
	prog = mp.MathematicalProgram()
	x = prog.NewContinuousVariables(2, "x")
	
	prog.AddLorentzConeConstraint(np.array([0*x[0]+1, x[0]-1, x[1]-1]))

	# prog.AddLinearConstraint(x[0] >= 1)
	# prog.AddLinearConstraint(x[1] >= 1)
	prog.AddQuadraticCost(np.eye(2), np.zeros(2), x)
	result = prog.Solve()

	print(result)
	assert result == mp.SolutionResult.kSolutionFound

	print(prog.GetSolution(x))
	assert np.allclose(prog.GetSolution(x), np.array([1, 1]))