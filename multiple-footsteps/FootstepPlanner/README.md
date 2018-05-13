# FootstepPlanner

This is a footstep planner for bipedal humanoid robots that I wrote as part of my UROP at the Robot Locomotion Group. It is based on [this](http://groups.csail.mit.edu/robotics-center/public_papers/Deits14a.pdf) paper, written by my research supervisor, Robin Deits.

![FootstepPlanner Output](/multiple-footsteps/FootstepPlanner/images/2D-AGGVQT.png?raw=true "FootstepPlanner Output")

## [FootstepPlanner](FootstepPlanner.py) ##
This is the main FootstepPlanner class. It has many methods to customize the various parameters of the footstep plan. These parameters include:
 - Reachable Regions: The region that the next footstep (left, right) can be, given where the previous (right, left - respectively) footstep was. This is important because, physically, the robot can only step so far given its current position.
   - `setReachableCircles()`: Use the intersection area of two circles, relative to the given footstep, to define the region where the next footstep can be.
   - `setReachableDiamonds()`: Use a convex hull, relative to the given footstep, to define the region where the next footstep can be.
 - Nominal Regions: _Only for Convex Reachable Regions (see above)._ Oftentimes, although a robot can step far from its current position, it would be better for stability for it to make smaller steps. In order to encode this relationship, we have a nominal ratio, to define a scaled down version of the reachable convex hull that is preferred for footsteps.
   - `setNominal()`
 - Obstacle Free Regions: Convex, obstacle free regions that define the safe regions for the robot to step in the local environment.
   - `setObstacleFree()`
 - Start/Goal: The starting position and goal position for the plan
   - `setStart()`: Set a single start position, around which the right and left footsteps will be centered.
   - `setStartRL()`: Set a starting position for the right and left footsteps.
   - `setGoal()`: Set a single goal position; the program will optimize to get as close as possible to this position.

The footstep planner also has several different methods to solve the resulting mathematical optimization problem. In particular, it uses two different solvers.
 - [Gurobi](http://www.gurobi.com/): Gurobi is a commercial solver that can handle mixed-integer quadratically constrained programs. It is wrapped into the Drake toolbox as a solver, and is used through pydrake. Although the Gurobi solver is very fast, it is not able to handle the angular constraints exactly, as sinusoidal constraints are not convex. Thus, the angular constraints are linearly approximated in order to make the problem tractable for the solver; this sacrifices potential search space.
   - `solveProgram()`
 - [Couenne](https://projects.coin-or.org/Couenne): Couenne is an open source solver that can exactly handle mixed-integer nonlinear programs. It is used through Julia, because the JuMP package provides a very clean interface for programmatically calling to Couenne. This solver lets us directly set up the exact optimization problem, but it is not as fast as the convex approximation used by Gurobi.
   - `solveProgram_CouenneCircle()`
   - `solveProgram_CouenneDiamond()`
 
## [Plotter](Plotter.py) ##
Plotter is a custom plotting class that takes inputs from the FootstepPlanner class and creates a visual representation of the problem. It is essentially a custom wrapper for `matplotlib` that can specifically deal with any of the parameters that FootstepPlanner uses.

## [LCM_requester](LCM_requester.py), [LCM_server](LCM_server.py) ##
LCM_requester and LCM_server are Python scripts that allow the FootstepPlanner to link into the Atlas simulator built into Drake. In particular, LCM_server is a server that listens for LCM [footstep_plan_request](drc/lcmtypes/drc_footstep_plan_request_t.lcm) messages and responds with the proper footstep plan by calling to FootstepPlanner to build the plan and sending out an LCM [footstep_plan](drc/lcmtypes/drc_footstep_plan_t.lcm). LCM_requester is simply a test script that sends a request to LCM_server and displays the resulting plan.