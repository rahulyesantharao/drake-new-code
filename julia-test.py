import julia
j = julia.Julia()
j._call('using JuMP, AmplNLWriter')
j._call('m = Model(solver=AmplNLSolver("/home/rahuly/Data/Couenne-0.5.6/build/bin/couenne"))')
j._call('theta = pi/6')
#print('setup!')
j._call('px = 10*cos(theta)')
j._call('py = 10*sin(theta)')
j._call('@variable(m,c)')
j._call('@variable(m,s)')
j._call('@NLobjective(m, Min, (px-c)^2+(py-s)^2)')
j._call('@constraint(m, c^2+s^2==1)')
#print('program spec!')
j._call('solve(m)')
#print('done!')
print('\nDONE!\n')
print('(c,s) = ({},{})'.format(j.eval('getvalue(c)'),j.eval('getvalue(s)')))
print('expected: ({},{})'.format(j.eval('cos(theta)'), j.eval('sin(theta)')))
