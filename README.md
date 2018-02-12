# drake-new-code

# Footstep Planning Code for Drake #
Python code written for the new Drake distro.
Before using, `source setup.sh`. Then `python setup-test.py` to make sure pydrake and mathematical programming work correctly.

## Schedule ##

### Fall 2017 ###
- [x] Create basic 2D footstep planner that can create a path from a goal position to end position in unconstrained space
- [x] Make the reachable regions from footsteps into general convex regions
- [x] Add a nominal and reachable distinction (so it is much costlier to step outside of some scaled-down version of the reachable region)
- [x] Add an option to make the reachable region into multiple convex regions

### IAP 2018 ###
- [x] Use [LCM](http://lcm-proj.github.io/) to feed plans into the old MATLAB simulator (`runAtlasWalking.m`) to allow for simulation of created footstep plans.

### Spring 2018 ###
- [ ] Tune constants to make footstep plans reasonable for Atlas
- [ ] Link in [COUENNE](https://projects.coin-or.org/Couenne) Solver to solve angle constraints

## Project Components ##

## [multiple-footsteps](multiple-footsteps) ##

### [multiple-footsteps/FootstepPlanner](multiple-footsteps/FootstepPlanner) ###
A Python package that can create footstep plans based on various input parameters, including start position, end position, convex obstacle-free regions, and nominal footstep regions. See the [README](multiple-footsteps/FootstepPlanner/README.md) in the folder for more info.
![FootstepPlanner output](/multiple-footsteps/FootstepPlanner/images/2D-4LBRCN.png?raw=true "FootstepPlanner Output")

## [convex-region-optimization](convex-region-optimization) ##

### [2D-qhull.py](convex-region-optimization/2D-qhull.py) ###
Can solve for the closest point within a number of 2D regions to a goal point (run with -s to save the outputted image).
![2D-qhull.py output](/convex-region-optimization/images/2D-6UK6PM.png?raw=true "2D-qhull.py Output")

### [3D-qhull.py](convex-region-optimization/3D-qhull.py) ###
Can solve for the closest point within a number of 3D regions to a goal point (run with -s to save the outputted image).
![3D-qhull.py output](/convex-region-optimization/images/3D-W3TGPS.png?raw=true "3D-qhull.py Output")
